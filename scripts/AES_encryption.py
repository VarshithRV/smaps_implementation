from Crypto.Cipher import AES
from Crypto.Hash import HMAC, SHA256
from Crypto.Random import get_random_bytes
import os

data = 'secret data to transmit'.encode()
path = "/home/barracuda/catkin_ws/src/smaps_implementation/bin"

aes_key = get_random_bytes(16)
hmac_key = get_random_bytes(16)

cipher = AES.new(aes_key, AES.MODE_CTR)
ciphertext = cipher.encrypt(data)

hmac = HMAC.new(hmac_key, digestmod=SHA256)
tag = hmac.update(cipher.nonce + ciphertext).digest()

f_aes = open(os.path.join(path,"AES_key.bin"), "wb")
f_aes.write(aes_key)

f_hmac = open(os.path.join(path,"hmac_key.bin"), "wb")
f_hmac.write(hmac_key)

f = open(os.path.join(path,"encrypted.bin"),"wb")
f.write(tag)
f.write(cipher.nonce)
f.write(ciphertext)