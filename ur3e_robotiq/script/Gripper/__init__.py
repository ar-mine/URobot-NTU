import sys
import os

current_path = os.path.realpath(__file__)
current_dir = os.path.split(current_path)[0]
sys.path.append(current_dir)

from RobotiqHand import *


