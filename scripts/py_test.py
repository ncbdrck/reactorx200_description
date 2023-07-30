#!/bin/python3

# reward = "sparssde"
#
# if reward == "Dense" or reward == "dense":
#     print("Dense")
# elif reward == "Sparse" or reward == "sparse":
#     print("Sparse")
# else:
#     print("The given Reward Architecture not found. Default to Dense")
#
#
# pkiu = True
#
# if pkiu is False:
#     print("Flase")
# else:
#     print("True")


import rospy

# Define ANSI escape codes for different colors
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
ENDC = '\033[0m'

rospy.init_node('my_node')

# Log messages in different colors
rospy.loginfo(RED + "This message is in red." + ENDC)
rospy.loginfo(GREEN + "This message is in green." + ENDC)
rospy.loginfo(YELLOW + 'This message is in yellow.' + ENDC)
rospy.loginfo(BLUE + 'This message is in blue.' + ENDC)
