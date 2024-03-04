# this is a script that creates a PUF lookup files for each device
# n devices, and m CRP for each device

from Crypto.Random import get_random_bytes

n = 10
m = 10


# create a binary file that contains the PUF table
for i in range(n):
    f = open("PUF_" + str(i) + ".bin", "wb")
    for j in range(m):
        challenge = get_random_bytes(16)
        response = get_random_bytes(16)
        f.write(challenge + response)
    f.close()

