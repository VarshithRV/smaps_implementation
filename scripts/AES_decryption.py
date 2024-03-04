import sys
from Crypto.Cipher import AES
from Crypto.Hash import HMAC, SHA256

# Somehow, the receiver securely get aes_key and hmac_key
# encrypted.bin can be sent over an unsecure channel

f_aes = open("AES_key.bin","rb")
aes_key = f_aes.read()

f_hmac = open("hmac_key.bin", "rb")
hmac_key = f_hmac.read()

with open("encrypted.bin", "rb") as f:
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