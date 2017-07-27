#!/usr/bin/env python
import redis

def vec3dCmp(a, b):
    for i in range(3):
        if a[i] < b[i]:
            return True
        if a[i] > b[i]:
            return False
    return False

def main():
    # Scaling factor from web client values to redis server
    SCALING = 0.1;

    obstacles = set()

    rPub = redis.StrictRedis(host='localhost', port=6379, db=0)
    rSub = redis.StrictRedis(host='localhost', port=6380, db=0)

    highestID = rPub.get("highestID")
    obstacles.clear()

    # Loop through all points
    for id in range(int(highestID) + 1):
        # Read in point info in Redis
        keyPoint = "p:" + str(id)

        # Only consider points currently in world
        if (rPub.sismember("points", keyPoint)):
            # Only consider goal points and obstacles
            if (str(rPub.hget(keyPoint, "name")) == "PEPoint"):
                keyPointPosition = keyPoint + ":position"
                # position = SCALING * rPub.get(keyPointPosition)

                # Set goal position or add to obstacles based off attract state
                if (str(rPub.hget(keyPoint, "attract")) == "true"):
                    # x_des_ = position
                    pass
                else:
                    # obstacles.add(position);
                    pass

    # Publishes to robot controller
    rPub.publish("nrc-trajectory", "TEST")

if __name__ == "__main__":
    main()