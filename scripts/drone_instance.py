# this class is a single drone instance that has the following functions:
# define the device id
# define its neighbours
# ability to communicate with the neighbours


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
            if msg.type == "AUTH" and msg.destination == self.device_id and self.device_id is not msg.message_queue[-1]:
                print(self.device_id,":AUTH message received, fowarding to next link")
                msg_new = Packet()
                msg_new.source = self.device_id
                msg_new.type = "AUTH"
                msg_new.current = msg.current + 1
                msg_new.destination = msg.message_queue[msg_new.current+1]
                msg_new.data = msg.data
                msg_new.message_queue = msg.message_queue
                print(self.device_id,":Sending message: ", msg_new)
                self.send_message(msg_new.destination,msg_new)
                # process challenge here and create response here
                # processed_data = self.process_msg_data(msg)


                

            
            # Message forwarding leaf node case
            elif msg.type == "AUTH" and msg.destination == self.device_id and self.device_id == msg.message_queue[-1]:
                print(self.device_id,":AUTH message received, leaf node")
                print(self.device_id,": ",msg.source," said ", msg.data,"creating a response and sending it back")
                # reverse the queue and return the message
                # process challenge here and create a response here
                # make a response packet, revert the queue, reset current
                # send the message back to the BS
                msg_new = Packet()
                msg_new.source = self.device_id
                msg_new.type = "RESPONSE"
                msg_new.message_queue = msg.message_queue[::-1]
                msg_new.current = 0
                msg_new.destination = msg_new.message_queue[msg_new.current+1]
                msg_new.data = " This for BS, I am "+str(self.device_id)
                print(self.device_id,":Sending message : ", msg_new," to ",msg_new.destination)
                self.send_message(msg_new.destination,msg_new)
                # process msg.data here, authenticate and create response here
            
            elif msg.type == "AUTH" and msg.destination is not self.device_id:
                print(self.device_id,":AUTH message received, not for me")
                return
            
            elif msg.type == "RESPONSE" and msg.destination == self.device_id and self.device_id is not msg.message_queue[-1]:
                print(self.device_id,":RESPONSE message received")
                print(self.device_id,": ",msg.source," said ", msg.data)
                msg.current += 1
                msg_new = Packet()
                msg_new.source = self.device_id
                msg_new.type = "RESPONSE"
                msg_new.current = msg.current
                msg_new.destination = msg.message_queue[msg_new.current+1]
                msg_new.data = msg.data #modify here
                msg_new.message_queue = msg.message_queue
                print(self.device_id,":Sending message : ", msg_new," to ",msg_new.destination)
                self.send_message(msg_new.destination,msg_new)
                # process msg.data here, authenticate and create response here    

    
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
    
    
    args = rospy.myargv(argv=sys.argv)
    drone = Device(int(args[1]))
    rospy.spin()
    
    # print("Device ID : ",drone.get_device_id())
    # puf_table = drone.PUF_table
    
    # # get the first challenge in the puf table
    # challenge = list(puf_table.keys())[0]
    # response = puf_table[challenge]
    # print("CRP queried from the table : ",challenge, response)
    # response = drone.PUF(challenge)
    # print("PUF(challenge) = ", response)

    # # encryption and decryption test
    # print("Encryption and Decryption test")
    # message="hello world"
    # print("Plain text: ", message)
    # tag,nonce,ciphertext = drone.encrypt(message,challenge)
    # de_message = drone.decrypt(tag,nonce,ciphertext,challenge)
    # print("Decrypted plaintext : ",message)

    # print("Neigbour Device ids") 
    # links = drone.get_links()
    # print(links)
    
    # print("Message sending and receiving test, monitor in neighbour's terminal")
    # time.sleep(3)
    # message = Packet()
    # message.source = drone.get_device_id()
    # message.destination = 2
    # message.data = "Hello from "+str(drone.get_device_id())
    # print("Sending message: ", message.data)
    # drone.send_message(message.destination,message)

    rospy.spin()


    

    


        

