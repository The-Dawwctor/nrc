#!/usr/bin/env python
import redis
import json
import numpy as np
import scipy.linalg as sl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def bezierCurve(goals, b, n):
    points = []
    for i in xrange(n - 1):
        for t in [x/10.0 for x in xrange(10)]:
            prevDiv = (2./3) * b[i] + (1./3) * b[i+1]
            nextDiv = (1./3) * b[i] + (2./3) * b[i+1]
            points.append((1 - t) ** 3 * goals[i] + 3 * (1 - t) ** 2 * t * prevDiv
                + 3 * (1 - t) * t ** 2 * nextDiv + t ** 3 * goals[i+1])
    points.append(goals[-1])
    return points

def generateTrajectory(goals):
    points = []
    if len(goals) > 2:
        # Initialize spline (A) & interpolated points (s) matrix
        n = len(goals)
        A = np.zeros([n-2, n-2])
        s = 6 * goals[1:-1]
        s[0] -= goals[0]
        s[-1] -= goals[-1]
        b = [goals[0]]  # Bezier curve control points

        # Calculate matrices
        A[-1][-1] = 4
        for i in range(n-3):
            A[i][i] = 4
            A[i][i+1] = 1
            A[i+1][i] = 1 

        b.extend(sl.solve(A, s, True, False, True, True))
        b.append(goals[-1])

        # Manually calculate all bezier curve points
        points = bezierCurve(goals, b, n)

    elif len(goals) == 2:   # If only 2 points, just do a line
        for i in xrange(10):
            points.append(goals[0] + (goals[1] - goals[0]) * i/10.0)

        points.append(goals[1])

    else: # SHOULD NOT HAPPEN, PROBLEM IF IT GOES HERE
        print "ERROR:"
        print "Goals: " + str(goals)

    return np.array(points)

def main():
    rPub = redis.StrictRedis(host='localhost', port=6379)
    rSub = redis.StrictRedis(host='localhost', port=6379)
    pSub = rSub.pubsub()
    pSub.subscribe("nrc-world-state", "nrc-next-goal")

    POSITIONS_KEY = "nrc::optitrack::pos_rigid_bodies"
    ORIENTATIONS_KEY = "nrc::optitrack::ori_rigid_bodies"
    EE_KEY = "nrc::kuka_iiwa::tasks::ee_pos"

    interp = []
    optiGoals = []
    idx = 0;

    # Listen for all published world states
    for message in pSub.listen():
        if message["type"] == "message":
            # Refresh starting position
            startPos = [0, 0, 0]
            if rPub.exists(EE_KEY):
                startPos = [float(x) for x in rPub.get(EE_KEY).split()]

            # Give next point in current trajectory
            if message["channel"] == "nrc-next-goal":
                idx += 1
                # Print current selected point in trajectory
                if idx < len(interp):
                    rPub.publish("nrc-trajectory", str(interp[idx].tolist()))

            # Creating new trajectory from Optitrack
            elif message["channel"] == "nrc-optitrack":
                if message["data"] == "start":
                    # Convert list of rigidbodies to go to into list of corresponding positions
                    # Accounts for gaps in number rigid body assignment
                    bodies = json.loads(rPub.get(POSITIONS_KEY))
                    rotates = json.loads(rPub.get(ORIENTATIONS_KEY))
                    
                    indices = [g[0] for g in sorted(enumerate(optiGoals), key=lambda x:x[1])]
                    optiPos = [bodies[i-1] for i in indices]
                    optiOrient = [rotates[i-1] for i in indices]

                    interp = generateTrajectory(optiPos)
                    idx = 0
                    optiGoals = []
                    rPub.publish("nrc-trajectory", str(interp[idx].tolist()))
                else:
                    optiGoals.append(int(message["data"]))

            # Creating new trajectory from web client
            elif message["channel"] == "nrc-world-state":
                decode = json.loads(message["data"])
                goals = []
                obstacles = []

                # Scaling factor from web client values to robot controller
                SCALING = 0.1

                # Loop through all points
                for point in decode:
                    # Only consider goal points and obstacles
                    if "Goal" in point[0]:
                        goals.append(point)
                    elif "Obstacle" in point[0]:
                        position = [SCALING * point[2], SCALING * point[3], SCALING * point[4]]
                        obstacles.append(position)

                goals.sort(key = lambda x : int(x[7])) # sorts points by order
                goals = [[SCALING * x[2], SCALING * x[3], SCALING * x[4]] for x in goals]

                # print goals

                goals.insert(0, startPos)
                goals = np.array(goals)

                idx = 0
                interp = generateTrajectory(goals)
                rPub.publish("nrc-trajectory", str(interp[idx].tolist()))
                rPub.publish("nrc-obstacles", str(obstacles))

                # Spline smoothness verification graph
                # fig = plt.figure()
                # ax = fig.add_subplot(111, projection='3d')
                # ax.plot(interp.T[0], interp.T[1], interp.T[2])
                # ax.plot(goals.T[0], goals.T[1], goals.T[2])
                # plt.show()

if __name__ == "__main__":
    main()