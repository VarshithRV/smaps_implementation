# this is a script that creates a PUF lookup files for each device
# n devices, and m CRP for each device

from Crypto.Random import get_random_bytes
import os, sys

args = sys.argv

n = 10
m = 10

if len(args) == 3:
    n = int(args[1])
    m = int(args[2])
else:
    print("Usage: python3 PUF_generator.py [number_of_devices] [number_of_CRPs]")
    print("Default: 10 devices, 10 CRPs")
    # sys.exit(1)

path = "/home/barracuda/catkin_ws/src/smaps_implementation/PUF_lookup"

for i in range(n):
    f = open(os.path.join(path,"PUF_" + str(i) + ".bin"), "wb")
    for j in range(m):
        challenge = get_random_bytes(16)
        response = get_random_bytes(16)
        f.write(challenge + response)
    f.close()

