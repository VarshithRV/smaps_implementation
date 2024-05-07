# Scalable Mutual Authentication Protocol for Drone Swarm systems

This work is the simulation of the SMAPS Protocol.

The SMAPS Protocol is a scalable mutual authentication protocol for all nodes in a drone swarm system and a Central Base Station. The Base Station is assumed to be Secure. <br>

The protocol uses Physical Unclonable Functions which are logic circuits that generates Challenge Response Pairs by utilizing the delays in the integrated circuit of the system. The above protocol uses a 128 bit Challenge and Response. The protocol uses the Challenge and Response as keys for encryption and decryption hence assumes that PUF is accurate down to the bit.

---
### PUF : Physical Unlonable Function
The PUF accpets 128 bit as a challenge and generates 128 bit Reponse for the corresponding challenge. <br>
- Given an IC, unique challenge will produce a unique Response.
- Given an IC, the same challenge will produce the same corresponding Response.
- Given a Challenge, Different IC produce different Reponses. A good PUF produces responses that are atleast 64 Hamming distance away from the reponse of any other IC for the same challenge.

The current simulation uses pregenerated PUF CRP for each drones. Any number of Pregenerated PUFs can be faciliated using the PUF_generator.py script, the PUFs are stored as a binary file in the PUF_lookup directory, this is later referred by the individual nodes. The current implementation of drone nodes refer the corresponding PUF file only, it is indentified using the drone_id, the base station has access to all the PUF CRP in the directory.<br>

---
### Protocol description
The protocol authenticates a given system and produces a list of legitimate and illegitimate nodes.<br>
The BS initiates the protocol, sends the aggregate Challenges for all the paths generated using the minimum spanning tree, when a message is received by a drone, the callback is activated that performes the necessary tasks like generating a response and further communication. So the protocol implementation is composed of the **Protocol Driver** in the Base Station and **Callback Functions** in all the drone instances.

#### Protocol workflow 
1. Network topology configuration in the `config/links.yaml` and the launch file configuration in the `launch/mesh.launch`.
2. Node launching and network initialization by launching the `launch/mesh.launch` file.
3. MST Creation and Paths finding by the BS.
4. Aggregate Challenge Message creation for each path by the BS.
5. Send the Aggregate Challenge to all the first elements of the Paths.
6. Message Communication Phase : 
   1. If node is not leaf, do : Authenticate BS, generate response, wait for response for child, concat node response with the child response.
   2. If node is leaf, do : Authenticate BS, generate response, send to message source.
7. BS collect all responses, Authenticates all of the nodes and generates sessions key.

#### Capabilities of Drone instance Class
1. Corresponding PUF CRP lookup.
2. AES Encryption, Decryption.
3. SHA256 Hashing.
4. Send(link,message).
5. Authenticate Base Station for all `AUTH` type messages.

#### Capabilites of Base Station Class
1. Global PUF CRP lookup.
2. AES Encryption, Decryption.
3. SHA256 Hashing.
4. Send(link,message).
5. Authenticate all nodes, classify into legitimate and illegitimate lists.

---
## Device class reference

#### Instance Variables
1. self.device_id - Int, stores the device id of the node.
2. self.links - List, stores all the neighbour device ids.
3. self.random - Bytes, contains a 128 bit random number.
4. self.pub_links_dict - Dictionary, self.pub_links_dict[link] contains the publisher for that link.
5. self.PUF_table - Dictionary that contains challenges as keys and responses as values.

### Instance Methods
1. self.send_message(link,message:Packet), return Bool success - sends the message if the link exits.
2. self.encrypt(self,message:string,key), return tag,nonce,ciphertext - encrypts and returns the tag, nonce and ciphertext.
3. self.decrypt(self,tag,nonce,ciphertext,key), return string message - decrypts and returns the string message.
4. self.PUF(challenge), return Response:Bytes, returns Response if Challenge exists.