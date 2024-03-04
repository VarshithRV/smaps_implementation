from Crypto.Cipher import AES
from Crypto.Hash import HMAC, SHA256
from Crypto.Random import get_random_bytes

data = 'secret data to transmit'.encode()

aes_key = get_random_bytes(16)
hmac_key = get_random_bytes(16)

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