Random_number_duration =   6.198883056640625e-06
puf_duration = 2.1457672119140625e-06 
encryption_duration = 0.0019729137420654297 
decryption_duration = 0.00276947021484375 
random_duration = 6.198883056640625e-06
Authentication_duration_single_drone = 2*Random_number_duration + puf_duration + encryption_duration + decryption_duration + random_duration
 
spanning_tree5 = 5.0067901611328125e-06 
spanning_tree10 = 3.5762786865234375e-06 
spanning_tree15 = 5.0067901611328125e-06 
spanning_tree20 = 5.245208740234375e-06 
spanning_tree25 = 6.67572021484375e-06 
spanning_tree30 = 8.58306884765625e-06

 
Authentication5 = 0.150067901611328125
Authentication10 = 0.185762786865234375
Authentication15 = 0.200067901611328125
Authentication20 = 0.27245208740234375
Authentication25 = 0.2867572021484375
Authentication30 = 0.3058306884765625

# put the communication_time, decryption_duration, Randome_number_duration, puf_duration, encryption_duration, decryption_duration, random_duration, spanning_tree_duration, Authentication_duration in a bar graph using matplotlib
import matplotlib.pyplot as plt

# Given values
communication_time = 1.1682510375976562e-05
decryption_duration = 0.00276947021484375
random_number_duration = 6.198883056640625e-06
puf_duration = 2.1457672119140625e-06
encryption_duration = 0.0019729137420654297

# Variable names and their corresponding durations
data1 = {
    
    'Decrypt': decryption_duration,
    'Authentication': Authentication_duration_single_drone,   
    'Encrypt': encryption_duration
}

data2 = {
    'PNRG': random_number_duration,
    'PUF': puf_duration,
    'COMM': communication_time
}

# Plotting the bar chart
plt.bar(data1.keys(), data1.values())
plt.title('Durations of Variables')
plt.xlabel('Variable Names')
plt.ylabel('Duration')
plt.xticks(rotation=45, ha="right") # Rotate x-axis labels for better readability
# plt.show()
#save the graph as a png file
plt.savefig('durations_of_variables1.png')
# do the same thing for data2
# clear the current figure
plt.clf()
plt.bar(data2.keys(), data2.values())
plt.title('Durations of Variables')
plt.xlabel('Variable Names')
plt.ylabel('Duration')
# decrease the size of the x-axis labels and rotate for better readability
plt.xticks(rotation=45, ha="right")
# plt.show()
#save the graph as a png file
plt.savefig('durations_of_variables2.png')
plt.clf()

# do the same thing for spanning tree durations
spanning_tree_durations = {
    '5': spanning_tree5,
    '10': spanning_tree10,
    '15': spanning_tree15,
    '20': spanning_tree20,
    '25': spanning_tree25,
    '30': spanning_tree30
}
plt.bar(spanning_tree_durations.keys(), spanning_tree_durations.values())
plt.title('Spanning Tree Durations')
plt.xlabel('Number of Nodes')
plt.ylabel('Duration')
plt.xticks(rotation=45, ha="right")
# plt.show()
#save the graph as a png file
plt.savefig('spanning_tree_durations.png')

plt.clf()
# do the same thing for Authentication durations
Authentication_durations = {
    '5': Authentication5,
    '10': Authentication10,
    '15': Authentication15,
    '20': Authentication20,
    '25': Authentication25,
    '30': Authentication30
}
plt.bar(Authentication_durations.keys(), Authentication_durations.values())
plt.title('Authentication Durations')
plt.xlabel('Number of Nodes')
plt.ylabel('Duration')
plt.xticks(rotation=45, ha="right")
# plt.show()
#save the graph as a png file
plt.savefig('Authentication durations.png')

