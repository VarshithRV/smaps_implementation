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

class Drone:
    
    
    def __init__(self,args = sys.argv):
        if len(args) != 2:
            print("Usage: python3 drone_instance.py [device_id]")
            sys.exit(1)
        
        
        # initialize the project path
        self.path = "/home/barracuda/catkin_ws/src/smaps_implementation"
        self.PUF_lookup_path = os.path.join(self.path, "PUF_lookup")
        self.config_path = os.path.join(self.path, "config")
        self.bin_path = os.path.join(self.path, "bin")
        
        # get the device id
        ###################
        self.device_id = int(args[1])
        
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
        rospy.spin()
    

    def create_links(self):
        for link in self.links:
            if link>self.device_id:
                pub =  rospy.Publisher("s"+str(link)+"x"+str(self.device_id)+"s", String, queue_size=10)
                self.publisher_list.append(pub)
                self.pub_links_dict[link] = pub
                rospy.Subscriber("s"+str(link)+"x"+str(self.device_id)+"s", String, self.msgCb)
            else:
                pub =  rospy.Publisher("s"+str(self.device_id)+"x"+str(link)+"s", String, queue_size=10)
                self.publisher_list.append(pub)
                self.pub_links_dict[link] = pub
                rospy.Subscriber("s"+str(self.device_id)+"x"+str(link)+"s", String, self.msgCb)
        # for link in self.links:
        #     pub =  rospy.Publisher("s"+str(link)+"x"+str(self.device_id)+"y"+str(self.device_id)+"x"+str(link)+"s", String, queue_size=10)
        #     self.publisher_list.append(pub)
        #     self.pub_links_dict[link] = pub
        #     rospy.Subscriber("s"+str(link)+"x"+str(self.device_id)+"y"+str(self.device_id)+"x"+str(link)+"s", String, self.msgCb)
    

    def msgCb(self,msg:String):
        print("Received message: ", msg.data)
    
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
    
    def PUF(self,challenge):
        if challenge in self.PUF_table:
            return self.PUF_table[challenge]
        else:
            return None
        
    def encrypt(self,message,key):

        data = message.encode()
        aes_key = key
        hmac_key = key

        cipher = AES.new(aes_key, AES.MODE_CTR)
        ciphertext = cipher.encrypt(data)

        hmac = HMAC.new(hmac_key, digestmod=SHA256)
        tag = hmac.update(cipher.nonce + ciphertext).digest()

        return tag,cipher.nonce,ciphertext

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


if __name__ == "__main__":
    
    
    drone = Drone()



    # # print(drone.get_device_id())
    # puf_table = drone.PUF_table
    
    # # get the first challenge in the puf table
    # challenge = list(puf_table.keys())[0]
    # response = drone.PUF(challenge)
    
    # # encryption and decryption test
    # message="hello world"
    # tag,nonce,ciphertext = drone.encrypt(message,challenge)
    # de_message = drone.decrypt(tag,nonce,ciphertext,challenge)
    # print(message)


    

    


        

