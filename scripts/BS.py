from drone_instance import Device
import os, sys, yaml
import rospy
from std_msgs.msg import String
from Crypto.Cipher import AES
from Crypto.Hash import HMAC, SHA256
from Crypto.Random import get_random_bytes
import time
from smaps_implementation.msg import Packet

# create a class called base station that inherits from the drone class
class BS(Device):
    def __init__(self, args = sys.argv):
        super().__init__(0)


if __name__ == "__main__":
    bs = BS()
    # bs.run()
    print("Base Station is running...")