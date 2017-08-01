#!/usr/bin/env python
import redis
import json
import numpy as np
import scipy.linalg as sl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def bezierCurve(s_p, s_n, b_p, b_n, t):
    prevDiv = (2./3) * b_p + (1./3) * b_n
    nextDiv = (1./3) * b_p + (2./3) * b_n
    return ((1 - t) ** 3 * s_p + 3 * (1 - t) ** 2 * t * prevDiv
        + 3 * (1 - t) * t ** 2 * nextDiv + t ** 3 * s_n)

def main():
    goals = list()
    obstacles = list()

    rPub = redis.StrictRedis(host='localhost', port=6379, db=0)
    rSub = redis.StrictRedis(host='localhost', port=6379, db=1)
    pSub = rSub.pubsub()
    pSub.subscribe("nrc-world-state")

    # Listen for all published world states
    for message in pSub.listen():
        if message["type"] == "message":
            decode = json.loads(message["data"])
            decode.sort(key = lambda x : int(x[1])) # sorts points by id

            # CLear list of goals and obstacles for each new message
            goals = []
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

            # Perform natural cubic spline interpolation
            # Initialize clamped spline (A) & interpolated points (s) matrix
            n = len(goals)
            A = np.empty([n, n])
            s = np.empty([n, 3])
            
            # Calculate matrices
            for i in range(n):
                edge = i == 0 or i == n-1
                for j in range(n):
                    if i == j:
                        A[i][j] = 2 if edge else 4
                    elif abs(i - j) == 1:
                        A[i][j] = 1
                    else:
                        A[i][j] = 0
                
                if edge:
                    s[i] = [3 * x for x in goals[i]]
                else:
                    s[i] = [6 * x for x in goals[i]]

            b = sl.solve(A, s, True, False, True, True)

            # Manually calculate all bezier curve points
            goals = np.array(goals)
            interp = []
            for i in xrange(n - 1):
                for j in xrange(10):
                    interp.append(bezierCurve(goals[i], goals[i+1], b[i], b[i+1], j/10.0))
            interp.append(goals[-1])

            # Spline smoothness verification
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            interp = np.array(interp)
            ax.plot(interp.T[0], interp.T[1], interp.T[2])
            ax.plot(goals.T[0], goals.T[1], goals.T[2])
            plt.show()

            # Publishes goals to robot controller
            goalStr = " ".join(str(x) for x in interp)
            obstacleStr = " ".join(str(x) for x in obstacles)
            result = "goals " + goalStr + " obstacles " + obstacleStr
            rPub.publish("nrc-trajectory", result)

if __name__ == "__main__":
    main()