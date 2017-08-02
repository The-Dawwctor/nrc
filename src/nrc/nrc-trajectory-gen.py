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
            # Creating new trajectory
            if message["channel"] == "nrc-world-state":
                decode = json.loads(message["data"])
                decode.sort(key = lambda x : int(x[1])) # sorts points by id

                # Refresh list of goals and obstacles for each new message
                eeKey = "nrc::kuka_iiwa::tasks::ee_pos"
                startPos = [float(x) for x in rPub.get(eeKey).split()] if rPub.exists(eeKey) \
                                                                        else [0, 0, 0]
                goals = [startPos]
                obstacles = []

                # Loop through all points
                for point in decode:
                    # Only consider goal points and obstacles
                    if "PEPoint" in point[0]:
                        # Scaling factor from web client values to robot controller
                        SCALING = 0.1
                        position = [SCALING * point[2], SCALING * point[3], SCALING * point[4]]

                        # Set goal position or add to obstacles based off attract state
                        if (point[7]):
                            goals.append(position)
                        else:
                            obstacles.append(position)

                # Initialize clamped spline (A) & interpolated points (s) matrix
                n = len(goals)
                A = np.zeros([n, n])
                goals = np.array(goals)
                s = 3 * goals[:]

                # Calculate matrices
                for i in range(n):
                    if i != n-1:
                        A[i][i] += 2
                        A[i][i+1] += 1
                        A[i+1][i] += 1
                        A[i+1][i+1] += 2
                    
                        if i != 0:
                            s[i] *= 2

                b = sl.solve(A, s, True, False, True, True)

                # Manually calculate all bezier curve points
                interp = np.asarray(bezierCurve(goals, b, n))
                idx = 0

                # Spline smoothness verification graph
                # fig = plt.figure()
                # ax = fig.add_subplot(111, projection='3d')
                # ax.plot(interp.T[0], interp.T[1], interp.T[2])
                # ax.plot(goals.T[0], goals.T[1], goals.T[2])
                # plt.show()

            # Give next point in current trajectory
            if message["channel"] == "nrc-next-goal":
                idx += 1

            # Print current selected point in trajectory
            if idx < len(interp):
                rPub.publish("nrc-trajectory", str(interp[idx].tolist()))

if __name__ == "__main__":
    main()