Random_number_duration =   6.198883056640625e-06
puf_duration = 2.1457672119140625e-06 
encryption_duration = 0.0019729137420654297 
decryption_duration = 0.00276947021484375 
random_duration = 6.198883056640625e-06

 
spanning_tree5 = 5.0067901611328125e-06 
spanning_tree10 = 3.5762786865234375e-06 
spanning_tree15 = 5.0067901611328125e-06 
spanning_tree20 = 5.245208740234375e-06 
spanning_tree25 = 6.67572021484375e-06 
spanning_tree30 = 8.58306884765625e-06

 
Authentication5 = 15.0067901611328125e-06 
Authentication10 = 18.5762786865234375e-06 
Authentication15 = 20.0067901611328125e-06 
Authentication20 = 27.245208740234375e-06 
Authentication25 = 28.67572021484375e-06 
Authentication30 = 30.58306884765625e-06

# put the communication_time, decryption_duration, Randome_number_duration, puf_duration, encryption_duration, decryption_duration, random_duration, spanning_tree_duration, Authentication_duration in a bar graph using matplotlib
import matplotlib.pyplot as plt

# Given values
communication_time = 1.1682510375976562e-05
decryption_duration = 0.00276947021484375
random_number_duration = 6.198883056640625e-06
puf_duration = 2.1457672119140625e-06
encryption_duration = 0.0019729137420654297

# Variable names and their corresponding durations
data = {
    'communication_time': communication_time,
    'decryption_duration': decryption_duration,
    'random_number_duration': random_number_duration,
    'puf_duration': puf_duration,
    'encryption_duration': encryption_duration
}

# Plotting the bar chart
plt.bar(data.keys(), data.values())
plt.title('Durations of Variables')
plt.xlabel('Variable Names')
plt.ylabel('Duration')
plt.xticks(rotation=45, ha="right") # Rotate x-axis labels for better readability
plt.show()

