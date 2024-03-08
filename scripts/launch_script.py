import os, sys, yaml
import rospy
from std_msgs.msg import String
from Crypto.Cipher import AES
from Crypto.Hash import HMAC, SHA256
from Crypto.Random import get_random_bytes
import time
from smaps_implementation.msg import Packet
from drone_instance import Device

if __name__ == "__main__":
    # rospy.init_node("drone_0", anonymous=True)
    # print("Drone is running...")
    args = rospy.myargv(argv=sys.argv)
    drone = Device(int(args[1]))

    print("Device ID : ",drone.get_device_id())
    puf_table = drone.PUF_table
    
    # get the first challenge in the puf table
    challenge = list(puf_table.keys())[0]
    response = puf_table[challenge]
    print("CRP queried from the table : ",challenge, response)
    response = drone.PUF(challenge)
    print("PUF(challenge) = ", response)

    # encryption and decryption test
    print("Encryption and Decryption test")
    message="hello world"
    print("Plain text: ", message)
    tag,nonce,ciphertext = drone.encrypt(message,challenge)
    de_message = drone.decrypt(tag,nonce,ciphertext,challenge)
    print("Decrypted plaintext : ",message)

    print("Neigbour Device ids") 
    links = drone.get_links()
    print(links)
    
    print("Message sending and receiving test, monitor in neighbour's terminal")
    time.sleep(3)
    message = Packet()
    message.source = drone.get_device_id()
    message.destination = 2
    message.data = "Hello from "+str(drone.get_device_id())
    print("Sending message: ", message.data)
    drone.send_message(message.destination,message)
    rospy.spin()
    # pass