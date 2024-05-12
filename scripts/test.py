import os, sys, yaml
import rospy
from std_msgs.msg import String
from Crypto.Cipher import AES
from Crypto.Hash import HMAC, SHA256
from Crypto.Random import get_random_bytes
import time
import heapq
from smaps_implementation.msg import Packet
import json

class Device:
    
    
    def __init__(self, device_id):
        
        # initialize the project path
        self.path = "/home/barracuda/catkin_ws/src/smaps_implementation"
        self.PUF_lookup_path = os.path.join(self.path, "PUF_lookup")
        self.config_path = os.path.join(self.path, "config")
        self.bin_path = os.path.join(self.path, "bin")
        
        # get the device id
        ###################
        self.device_id = device_id
        
        # initialize drone instance
        rospy.init_node("drone_"+str(self.device_id), anonymous=True)
        
        # create a PUF_lookup directory and dictionary
        ###################
        self.PUF_table = {}
        f = open(os.path.join(self.PUF_lookup_path,"PUF_"+str(self.device_id)+".bin"), "rb")
        for i in range(10):
            challenge = f.read(16)
            response = f.read(16)
            self.PUF_table[challenge] = response
        f.close()

        # read the yaml file
        with open(os.path.join(self.config_path, "links.yaml"), "rb") as stream:
            try:
                self.config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        # initialize the links
        ###################
        self.links = self.config[self.device_id]

        # generate and store and random number for the device
        ###################
        self.random = get_random_bytes(16)

        # publisher list for links 
        ###################
        self.publisher_list = []

        # create a dictionary for the links with links as keys
        ###################
        self.pub_links_dict = {}

        # create the links
        self.create_links()
        # rospy.spin()

        # timestamp
        self.timestamp = time.time()
        print(self.pub_links_dict)

    ############################
    def create_links(self):
        for link in self.links:
            if link>self.device_id:
                pub =  rospy.Publisher("s"+str(link)+"x"+str(self.device_id)+"s", Packet, queue_size=10)
                self.publisher_list.append(pub)
                self.pub_links_dict[link] = pub
                rospy.Subscriber("s"+str(link)+"x"+str(self.device_id)+"s", Packet, self.msgCb)
            else:
                pub =  rospy.Publisher("s"+str(self.device_id)+"x"+str(link)+"s", Packet, queue_size=10)
                self.publisher_list.append(pub)
                self.pub_links_dict[link] = pub
                rospy.Subscriber("s"+str(self.device_id)+"x"+str(link)+"s", Packet, self.msgCb)
    ############################
    def send_message(self,link,message:Packet):
        if link not in self.links:
            print("Link not found")
            return False
        else:
            self.pub_links_dict[link].publish(message)
            return True
    ############################
    def encrypt(self,message,key):

        data = message.encode()
        aes_key = key
        hmac_key = key

        cipher = AES.new(aes_key, AES.MODE_CTR)
        ciphertext = cipher.encrypt(data)

        hmac = HMAC.new(hmac_key, digestmod=SHA256)
        tag = hmac.update(cipher.nonce + ciphertext).digest()

        return tag,cipher.nonce,ciphertext
    ############################
    def decrypt(self,tag,nonce,ciphertext,key):

        aes_key = key
        hmac_key = key

        try:
            hmac = HMAC.new(hmac_key, digestmod=SHA256)
            tag = hmac.update(nonce + ciphertext).verify(tag)
        except ValueError:
            print("The message was modified!")
            sys.exit(1)

        cipher = AES.new(aes_key, AES.MODE_CTR, nonce=nonce)
        message = cipher.decrypt(ciphertext)
        return message.decode()
    ############################
    def PUF(self,challenge):
        if challenge in self.PUF_table:
            return self.PUF_table[challenge]
        else:
            return None
    ############################
    
    def process_msg_data (self, msg):
        data_dict = json.loads(msg.data)
        challenge = data_dict["challenge"]
        timestamp = data_dict["timestamp"]
        random_number_enc = data_dict["random_number_enc"]
        tag = random_number_enc[0]
        nonce = random_number_enc[0]
        ciphertext = random_number_enc[0]
        timestamp_enc = data_dict["timestamp_enc"]
        response_enc = data_dict["response_enc"]

        return 

    def msgCb(self,msg:Packet):
        if msg.source == self.device_id:
            return
        else:
            print("##############################################################")
            # Message forwarding
            print(self.device_id,":Received :", msg,"from ",msg.source)

    
    def get_device_id(self):
        # print(self.device_id)
        return self.device_id
    
    def get_PUF_table(self):
        # print(self.PUF_table)
        return self.PUF_table
    
    def get_links(self):
        # print(self.links)
        return self.links
        # print(self.config)
    
    def get_random(self):
        return self.random
    
    def update_random(self):
        self.random = get_random_bytes(16)

    def get_config(self):    
        return self.config 


if __name__ == "__main__":
    device = Device(int(sys.argv[1]))
    # print("Device ID : ",device.get_device_id())
    # print("PUF Table : ",device.get_PUF_table())
    # print("Neigbouring Links : ",device.get_links())
    # print("Random number : ",device.get_random())
    # device.update_random()
    # print("New random : ",device.get_random())
    print("ID : ",device.device_id)
    print("PUF : ",device.PUF_table)
    print("Links : ",device.links)
    if device.device_id == 1:
        message = Packet()
        message.source = 1
        message.destination = 2
        message.data = "Hello"
        rospy.sleep(1)
        print(device.send_message(2,message))
        print(device.send_message(3,message))
    rospy.spin()
