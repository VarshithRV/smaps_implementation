import os, sys, yaml
import rospy
from Crypto.Cipher import AES
from Crypto.Hash import HMAC, SHA256
from Crypto.Random import get_random_bytes
import time
import heapq
from smaps_implementation.msg import Packet
import base64
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
        self.links = self.config[self.device_id] # confiugration dictionary

        # generate and store and random number for the device
        ###################
        self.random = get_random_bytes(16) # 16 Byte random number

        # publisher list for links 
        ###################
        self.publisher_list = []

        # create a dictionary for the links with links as keys
        ###################
        self.pub_links_dict = {} # dictionary containing publisher for each  link 

        # create the links
        self.create_links() # geneartes communication links
        # rospy.spin()

        # timestamp
        self.timestamp = time.time() # timestamp

        # auth responses list, an element contains the aggregate response of that path
        self.auth_responses = []

    
    
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
                
    
    def send_message(self,link,message:Packet): # sends message of type Packet to given node
        if link not in self.links:
            print("Link not found")
            return False
        else:
            self.pub_links_dict[link].publish(message)
            return True
    
    def encrypt(self,message,key):
        data = message
        aes_key = key
        hmac_key = key

        cipher = AES.new(aes_key, AES.MODE_CTR)
        ciphertext = cipher.encrypt(data)

        hmac = HMAC.new(hmac_key, digestmod=SHA256)
        tag = hmac.update(cipher.nonce + ciphertext).digest()

        return tag,cipher.nonce,ciphertext
   
    def decrypt(self,tag,nonce,ciphertext,key): # dencrypts, returns tag, nonce and ciphertext
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
    
    # PUF function 
    def PUF(self,challenge):
        if challenge in self.PUF_table:
            return self.PUF_table[challenge]
        else:
            return None
    
    # Just print what BS received
    def msgCb(self,msg:Packet):
        if msg.source == self.device_id:
            pass
        else:
            # print(self.device_id,":Received :", msg,"from ",msg.source)
            response = json.loads(msg.data)
            print(self.device_id,": Aggregate response received from one path : ",response)
            self.auth_responses.append(response)


    
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

def generate_mst(graph): # given config dict, returns a min spanning tree
    mst = {}
    visited = set()
    start_node = list(graph.keys())[0]
    heap = [(0, start_node, None)]  # (weight, current_node, parent_node)

    while heap:
        weight, current_node, parent_node = heapq.heappop(heap)

        if current_node not in visited:
            visited.add(current_node)

            if parent_node is not None:
                mst.setdefault(parent_node, []).append(current_node)

            for neighbor in graph[current_node]:
                if neighbor not in visited:
                    heapq.heappush(heap, (0, neighbor, current_node))

    return mst

def find_all_paths(graph, start, end, path=[]): # returns path of start and end
    path = path + [start]
    if start == end:
        return [path]
    paths = []
    for node in graph[start]:
        if node not in path:
            newpaths = find_all_paths(graph, node, end, path)
            for p in newpaths:
                paths.append(p)
    return paths

def find_leafs(mst): # returs a list of leaf nodes
    leafs = []
    for values in mst.values():
        for value in values:
            if value not in mst.keys():
                leafs.append(value)
    return leafs

def add_leafs(mst,leafs):
    for leaf in leafs:
        mst[leaf] = []    
    return mst

def find_paths(graph,root): # returns all paths for all leaf nodes
    mst = generate_mst(graph)
    leafs = find_leafs(mst)
    mst = add_leafs(mst,leafs)
    paths = []
    for leaf in leafs:
        path = find_all_paths(mst, root, leaf)
        paths.append(path)
    paths_new = []
    for path in paths:
        path = path[0]
        paths_new.append(path)
    return paths_new


##################################################
# Base Station class
class BS(Device):
    
    def __init__(self):
        super().__init__(0)
        
        # all drones list
        self.drones = list(self.config.keys())
        self.drones.remove(0)
        
        # PUF directory creation
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
        
        # getting list of paths
        self.paths = find_paths(self.config,self.device_id)
        
        # the first link of all paths
        self.starting_links = []
        for path in self.paths:
            self.starting_links.append(path[1])

        # creating the authentication messages for all drones
        self.auth_messages = self.create_auth_messages() #self.auth_messages[i] contains the authentication message for path i as a dictionary

        self.flatten_auth_messages = self.create_node_auth_messages() #self.flatten_auth_messages[i] contains the authentication message for drone i as a dictionary

    # get the nth CRP of the drone with id i as a list of challenge and response
    def get_PUF_CRP(self,drone_id,n):
        return list(self.PUF_directory[drone_id].keys())[n], list(self.PUF_directory[drone_id].values())[n]

    # generate the authentication message dictionary for all the drones in the topology
    def create_node_auth_messages(self):
        auth_messages = {}
        for drone in self.drones:
            # get the first challenge of each drone
            challenge = list(self.PUF_directory[drone].keys())[0] 
            response = self.PUF_directory[drone][challenge] 
            message = {}
            message['challenge'] = base64.b64encode(challenge).decode('utf-8')# convert challenge bytes to string
            tag,nonce,ciphertext = self.encrypt(base64.b64encode(response).decode('utf-8').encode(),response)
            tag =  base64.b64encode(tag).decode('utf-8')
            nonce = base64.b64encode(nonce).decode('utf-8')
            ciphertext = base64.b64encode(ciphertext).decode('utf-8')
            message['response_enc'] = [tag,nonce,ciphertext] # encrypted response, list of tag, nonce and ciphertext
            auth_messages[drone]=message
        return auth_messages
    
    # generate authentication message for each path
    def create_auth_messages(self):
        node_auth_messages = self.create_node_auth_messages()
        authentication_message = []
        i =0
        for path in self.paths:
            path_message = {}
            for node in path:
                if node == 0:
                    pass
                else :
                    path_message[node] = node_auth_messages[node]
            authentication_message.append(path_message)
            i+=1
        return authentication_message

    # decrypt all the responses received from the drones
    def decrypt_responses(self):
        for i in range(len(self.auth_responses)):
            for key in self.auth_responses[i].keys():
                response = self.auth_responses[i][key]
                tag = base64.b64decode(response[0])
                nonce = base64.b64decode(response[1])
                cipher_text = base64.b64decode(response[2])
                response_extracted = self.decrypt(tag,nonce,cipher_text,
                                                  self.PUF_directory[int(key)][base64.b64decode(self.flatten_auth_messages[int(key)]['challenge'])]
                                                  )
                self.auth_responses[i][key] = base64.b64decode(response_extracted)
            
    # Authenticate the drones in the topology
    def authenticate_drones(self):
        print("Response received from all the drones : ",self.auth_responses)

        legitimate = []

        for i in range(len(self.auth_responses)):
            for key in self.auth_responses[i].keys():
                if self.auth_responses[i][key] == self.PUF_directory[int(key)][base64.b64decode(self.flatten_auth_messages[int(key)]['challenge'])]:
                    print("Drone ",key," is authenticated")
                    print("Response received : ",self.auth_responses[i][key])
                    print("Expected response : ",self.PUF_directory[int(key)][base64.b64decode(self.flatten_auth_messages[int(key)]['challenge'])])
                    legitimate.append(key)
                else:
                    print("Drone ",key," is not authenticated")

        # remove duplicates from legitimate
        legitimate = list(set(legitimate))

        # convert all the elements in drone to str
        drones = list(map(str,self.drones))
        illegitimate = list(set(drones) - set(legitimate))
        
        return legitimate, illegitimate
    
def SMAPSSTAR_protocol(bs:BS):
    print(bs.device_id,": Base Station is running...")
    print(bs.device_id,": Paths : ",bs.paths)
    print(bs.device_id,": Starting links : ",bs.starting_links)
    print(bs.device_id,": Drones  : ",bs.drones)
    print(bs.device_id,": PUF directory : ",bs.PUF_directory)
    print(bs.device_id,": Sending AUTH message to all the neighbouring links of Base Station 0")
    print("Initializing the publishers ...")
    time.sleep(1)
    
    i =0
    for link in bs.starting_links:
        msg = Packet()
        msg.type = "AUTH"
        msg.source = 0
        msg.destination = link

        # send the auth message here
        msg.data = json.dumps(bs.auth_messages[i])

        msg.message_queue = bs.paths[i]
        print("########################################################")
        print(bs.device_id,": Sending message ",msg," to ",link)
        bs.send_message(link,msg)
        print(bs.device_id,": Message sent")
        i+=1

        time.sleep(0.05)
    
    # Wait for all responses here and assume that all responses are received
    time.sleep(5)

    print("########################################################")
    print("All responses received : ")
    
    print("Responses : ",bs.auth_responses)
    # get the decrypted responses from the messages
    bs.decrypt_responses()
    
    # Authenticating all the nodes in the topology
    legitimate, intermediate = bs.authenticate_drones()
    
    illegitimate = []

    # getting confirmed illegitimate drones
    paths = bs.paths
    
    for path in paths:
        for i in range(len(path)):
            path[i] = str(path[i])

    
    for path in paths:
        illegitimate_in_path = []
        for node in path:
            if node in intermediate:
                illegitimate_in_path.append(node)
                pass
        print("Path : ",path," Illegitimate drones : ",illegitimate_in_path)
        if len(illegitimate_in_path) > 0:
            illegitimate.append(illegitimate_in_path[0])
    
    illegitimate = list(set(illegitimate))
    
    drone_set = []
    for drone in bs.drones:
        drone_set.append(str(drone))
    drone_set = set(drone_set)
    unauthenticated = list(drone_set - set(legitimate) - set(illegitimate))

    return legitimate, illegitimate, unauthenticated


if __name__ == "__main__":
    bs = BS()
    args = rospy.myargv(argv=sys.argv)
    legitimate, illegitimate,  unauthenticated = SMAPSSTAR_protocol(bs)
    print("Legitimate drones : ",legitimate)
    print("Illegitimate drones : ",illegitimate)
    print("Unauthenticated drones : ",unauthenticated)