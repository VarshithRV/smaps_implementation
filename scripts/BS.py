import os, sys, yaml
import rospy
from std_msgs.msg import String
from Crypto.Cipher import AES
from Crypto.Hash import HMAC, SHA256
from Crypto.Random import get_random_bytes
import time
import heapq
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
        data = message
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
    
    
    def msgCb(self,msg:Packet):
        if msg.source == self.device_id:
            pass
        else:
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

def generate_mst(graph):
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

def find_all_paths(graph, start, end, path=[]):
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

def find_leafs(mst):
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

def find_paths(graph,root):
    mst = generate_mst(graph)
    leafs = find_leafs(mst)
    mst = add_leafs(mst,leafs)
    paths = []
    for leaf in leafs:
        path = find_all_paths(mst, root, leaf)
        paths.append(path)
    return paths

##################################################
# Base Station class
class BS(Device):
    
    def get_PUF_CRP(self,drone_id,n): # get the nth CRP of the drone with id i
        return list(self.PUF_directory[drone_id].keys())[n], list(self.PUF_directory[drone_id].values())[n]

    def create_node_auth_messages(self): # generates auth messages for drones, returns dict with node_id as key and auth_message as value
        # create the AUTH messsage for all the nodes
        # message = Challenge[0]+timestamp+Encrypt(random_number+timestamp+R[0],R[0])
        auth_messages = {}
        for drone in self.drones:
            # get the first challenge of each drone
            challenge = list(self.PUF_directory[drone].keys())[0]
            response = self.PUF_directory[drone][challenge]
            timestamp = time.time()
            self.update_random()
            random_number = self.random
            message = {}
            # message['challenge'] = challenge.decode()
            # print(challenge)
            # print(str(challenge),type(str(challenge)),str(challenge).encode(),type(str(challenge).encode()))
            message['challenge'] = challenge
            message['timestamp'] = timestamp
            message['random_number_enc'] = list(self.encrypt(random_number,response))
            message['timestamp_enc'] = list(self.encrypt(str(timestamp).encode(),response))
            message['response_enc'] = list(self.encrypt(response,response))
            # print("Message for drone ",drone,": ",message)
            auth_messages[drone]=message
        return auth_messages
    
    def create_auth_messages(self): # returns a list of dicts (the dicts contain aggregate message for that path)

        node_auth_messages = bs.create_node_auth_messages() #auth_message[i] contains the auth message for ith node
        authentication_message = [] # a list that contains aggregate path message as items, authentication_message[1] contains aggregate message for path 1
        i =0
        for path in bs.paths:
            path = path[0]
            path_message = {}
            for node in path:
                if node == 0:
                    pass
                else :
                    path_message[node] = node_auth_messages[node]
            authentication_message.append(path_message)
            i+=1
        return authentication_message


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
        # all drones list
        self.drones = list(self.config.keys())
        
        ##################################################
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
        
        ##################################################
        # initializing all paths
        self.paths = find_paths(self.config,self.device_id)
        print(self.paths)
        ##################################################
        # starting links for the paths
        self.starting_links = []
        for path in self.paths:
            self.starting_links.append(path[0][1])
        print(self.starting_links)
        ##################################################
            
if __name__ == "__main__":
    
    args = rospy.myargv(argv=sys.argv)
    bs = BS(args)
    print(bs.device_id,": Base Station is running...")
    # prepare the path messages
    auth_messages = bs.create_auth_messages()
    # print the paths
    print(bs.device_id,": Paths : ",bs.paths)
    # print the starting links
    print(bs.device_id,": Starting links : ",bs.starting_links)
    
    print(bs.device_id,": Sending AUTH message to all the neighbouring links of Base Station 0")

    for i in range(len(bs.starting_links)):
        print(bs.device_id,": Aggregate AUTH message of path i : ",auth_messages[i])

    i =0
    for link in bs.starting_links:
        msg = Packet()
        msg.type = "AUTH"
        msg.source = 0
        msg.destination = link
        msg.data = "Hello Drone "+str(link)+" from Base Station 0" # this is where you send the message
        msg.message_queue = bs.paths[i][0]
        print(bs.device_id,": Sending message ",msg," to ",link)
        bs.send_message(link,msg)
        print(bs.device_id,": Message sent")
        time.sleep(2)
        i+=1
    
    rospy.spin()
