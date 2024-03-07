# read it and store it in a dictionary
import os

path = "/home/barracuda/catkin_ws/src/smaps_implementation/PUF_lookup"

f = open(os.path.join(path,"PUF_0.bin"), "rb")
puf_table = {}
for i in range(10):
    challenge = f.read(16)
    response = f.read(16)
    puf_table[challenge] = response
f.close()


# # print all the challenges and responses
# for challenge, response in puf_table.items():
#     print(challenge, response)

# print the first challenge and response
print("First challenge and response")
print(list(puf_table.keys())[0], list(puf_table.values())[0])

# --------------------------------------------------------------------------------
# now use this for AES encryption
from Crypto.Cipher import AES
from Crypto.Hash import HMAC, SHA256
from Crypto.Random import get_random_bytes

data = 'secret data to transmit'.encode()

aes_key = list(puf_table.keys())[0]
hmac_key = list(puf_table.values())[0]

cipher = AES.new(aes_key, AES.MODE_CTR)
ciphertext = cipher.encrypt(data)

hmac = HMAC.new(hmac_key, digestmod=SHA256)
tag = hmac.update(cipher.nonce + ciphertext).digest()

f_aes = open("AES_key.bin", "wb")
f_aes.write(aes_key)

f_hmac = open("hmac_key.bin", "wb")
f_hmac.write(hmac_key)

f = open("encrypted.bin","wb")
f.write(tag)
f.write(cipher.nonce)
f.write(ciphertext)


