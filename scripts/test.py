
from Crypto.Random import get_random_bytes
import base64

# Generate random bytes
random_bytes = get_random_bytes(16)
original_bytes = b'?\x99K7\xdc\x92\x1d\x81\xb4\xce\x03Y\x9d=\xc3\xe9'

# Encode the bytes to a Base64 string
base64_string = base64.b64encode(original_bytes).decode('utf-8')

print(base64_string,type(base64_string))

# To convert it back to bytes
decoded_bytes = base64.b64decode(base64_string)

print(decoded_bytes,type(decoded_bytes))