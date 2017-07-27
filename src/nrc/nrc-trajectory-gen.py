#!/usr/bin/env python
import redis
import json

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
            decode.sort(key = lambda x : int(x[1]))

            # CLear list of goals and obstacles for each new message
            goals = []
            obstacles = []

            # Loop through all points
            for point in decode:
                # Only consider goal points and obstacles
                if "PEPoint" in point[0]:
                    # Scaling factor from web client values to robot controller
                    SCALING = 0.1
                    position = (SCALING * point[2], SCALING * point[3], SCALING * point[4])

                    # Set goal position or add to obstacles based off attract state
                    if (point[7]):
                        goals.append(position)
                    else:
                        obstacles.append(position)

        # Publishes goals to robot controller
        result = "goals:" + str(goals) + ";obstacles:" + str(obstacles)
        # rPub.publish("nrc-trajectory", result)

    pSub.close()

if __name__ == "__main__":
    main()