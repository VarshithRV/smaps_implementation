# import os library
import os

# read the arguments provided to the file at runtime
import sys
# get the arguments provided at runtime
args = sys.argv

print(args)
print(type(args),len(args),args[1])

# -------------------------------------------------------------------------------
# create a path to the PUF_lookup directory
# path = os.path.join(os.getcwd(), "PUF_lookup")

# open a file in the PUF_lookup directory in write mode

path = "/home/barracuda/catkin_ws/src/smaps_implementation/PUF_lookup"

f = open(os.path.join(path, "PUF_0.bin"), "wb")

path = "/home/barracuda/catkin_ws/src/smaps_implementation/config"

#--------------------------------------------------------------------------------
# reading yaml files
import yaml
with open(os.path.join(path, "file1.yaml"), "rb") as stream:
    try:
        puf_table = yaml.safe_load(stream)
        print(puf_table)
    except yaml.YAMLError as exc:
        print(exc)
# get the first element in the dictionary

print(type(puf_table),type(puf_table[1]),puf_table[1])

