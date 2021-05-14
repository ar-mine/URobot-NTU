import sys
import os

current_path = os.path.realpath(__file__)
current_dir = current_path[:-11]
sys.path.append(current_dir)

from RobotiqHand import *


