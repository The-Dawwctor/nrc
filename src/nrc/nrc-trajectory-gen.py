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

def main():
    rPub = redis.StrictRedis(host='localhost', port=6379, db=0)
    rSub = redis.StrictRedis(host='localhost', port=6379, db=1)
    pSub = rSub.pubsub()
    pSub.subscribe("nrc-world-state", "nrc-next-goal")

    interp = []
    idx = 0;

    # Listen for all published world states
    for message in pSub.listen():
        if message["type"] == "message":
            # Give next point in current trajectory
            if message["channel"] == "nrc-next-goal":
                idx += 1
            # Creating new trajectory
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

                print goals

                # Refresh list of goals and obstacles for each new message
                eeKey = "nrc::kuka_iiwa::tasks::ee_pos"
                startPos = [float(x) for x in rPub.get(eeKey).split()] if rPub.exists(eeKey) else [0, 0, 0]
                goals.insert(0, startPos)
                goals = np.array(goals)

                if len(goals) > 2:
                    # Initialize clamped spline (A) & interpolated points (s) matrix
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
                    interp = bezierCurve(goals, b, n)
                elif len(goals) == 2:   # If only 2 points, just do a line
                    for i in xrange(10):
                        interp.append(goals[0] + (goals[1] - goals[0]) * i/10.0)

                    interp.append(goals[1])
                else: # SHOULD NOT HAPPEN, PROBLEM HERE
                    print "Points: " + str(decode)
                    print "Goals: " + str(goals)

                idx = 0
                interp = np.array(interp)
                # Spline smoothness verification graph
                # fig = plt.figure()
                # ax = fig.add_subplot(111, projection='3d')
                # ax.plot(interp.T[0], interp.T[1], interp.T[2])
                # ax.plot(goals.T[0], goals.T[1], goals.T[2])
                # plt.show()

            # Print current selected point in trajectory
            if idx < len(interp):
                rPub.publish("nrc-trajectory", str(interp[idx].tolist()))

if __name__ == "__main__":
    main()