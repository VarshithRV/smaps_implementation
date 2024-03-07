import sys, os
from Crypto.Cipher import AES
from Crypto.Hash import HMAC, SHA256

# Somehow, the receiver securely get aes_key and hmac_key
# encrypted.bin can be sent over an unsecure channel

path = "/home/barracuda/catkin_ws/src/smaps_implementation/bin"
f_aes = open(os.path.join(path,"AES_key.bin"),"rb")
aes_key = f_aes.read()

f_hmac = open(os.path.join(path,"hmac_key.bin"), "rb")
hmac_key = f_hmac.read()

with open(os.path.join(path,"encrypted.bin"), "rb") as f:
    tag = f.read(32)
    nonce = f.read(8)
    ciphertext = f.read()

try:
    hmac = HMAC.new(hmac_key, digestmod=SHA256)
    tag = hmac.update(nonce + ciphertext).verify(tag)
except ValueError:
    print("The message was modified!")
    sys.exit(1)

cipher = AES.new(aes_key, AES.MODE_CTR, nonce=nonce)
message = cipher.decrypt(ciphertext)
print("message : ", message)
print("Message:", message.decode())