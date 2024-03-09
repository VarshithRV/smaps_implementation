import os, sys, yaml
import rospy
from std_msgs.msg import String
from Crypto.Cipher import AES
from Crypto.Hash import HMAC, SHA256
from Crypto.Random import get_random_bytes
import time
from smaps_implementation.msg import Packet

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
    
    def send_message(self,link,message:Packet):
        if link not in self.links:
            print("Link not found")
            return False
        else:
            self.pub_links_dict[link].publish(message)
            return True


    def msgCb(self,msg:Packet):
        if msg.source == self.device_id:
            pass
        else:
            print(self.device_id,": ", msg.data)
    
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
    

class BS(Device):
    
    def get_PUF_CRP(self,drone_id,n): # get the nth CRP of the drone with id i
        return list(self.PUF_directory[drone_id].keys())[n], list(self.PUF_directory[drone_id].values())[n]

    def __init__(self, args):

        super().__init__(0)

        ##################################################
        # list all attributes from the parent class
        # 1. device_id - 0
        # 2. PUF_table - table of device 0s CRP 
        # 3. config - topology
        # 4. links - neighbours of device 0
        # 5. random - random number for device 0
        # 6. pub_links_dict - dictionary of publishers for the links
        # 7. self.path = "/home/barracuda/catkin_ws/src/smaps_implementation"
        # 8. self.PUF_lookup_path = os.path.join(self.path, "PUF_lookup")
        # 9. self.config_path = os.path.join(self.path, "config")
        # 10. self.bin_path = os.path.join(self.path, "bin")

        # method from the parent class
        # 1. Bool send_message(link,message:Packet) - send message to a link
        # 2. void msgCb(msg:Packet) - callback function for message
        # 3. int get_device_id() - get the device id
        # 4. dictionary get_PUF_table() - get the PUF table of device 0
        # 5. list get_links() - get the links of device 0
        # 6. bytes PUF(challenge) - get the response from the PUF table of device 0
        # 7. tag, nonce, ciphertext encrypt(message,key) - encrypt the message
        # 8. string message decrypt(tag,nonce,ciphertext,key) - decrypt the message
        ##################################################
        
        self.drones = list(self.config.keys())
        
        ##################################################
        
        self.PUF_directory = {}
        for i in self.drones: 
            PUF_table_temp = {}
            f = open(os.path.join(self.PUF_lookup_path,"PUF_"+str(i)+".bin"), "rb")
            
            for j in range(10):
                challenge = f.read(16)
                response = f.read(16)
                PUF_table_temp[challenge] = response
            f.close()
            
            self.PUF_directory[i] = PUF_table_temp
        
        ##################################################
            


            
if __name__ == "__main__":
    
    args = rospy.myargv(argv=sys.argv)
    bs = BS(args)
    print("Base Station is running...")
    rospy.spin()