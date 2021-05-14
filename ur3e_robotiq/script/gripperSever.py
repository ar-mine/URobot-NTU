# -*- coding: utf-8 -*-
import time
import signal
import sys
import os

current_path = os.path.realpath(__file__)
current_dir = os.path.split(current_path)[0]
sys.path.append(current_dir)
from Gripper import RobotiqHand

import rospy
from sensor_msgs.msg import JointState

HOST = "192.168.0.111"
PORT = 54321

rospy.init_node('gripper_sever')

joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
joint_state = JointState()
joint_state.name.append("finger_joint")
joint_state.position.append(0)

rate = rospy.Rate(50) # 50hz

def handler(signal, frame):
    exit(0)

def gripper_init(hand):
    print("Connect finish")
    print('activate: start')
    hand.reset()
    hand.activate()
    result = hand.wait_activate_complete()
    if result != 0x31:
        return 1
    else:
        return 0

def main():
    global HOST, PORT 
    hand = RobotiqHand()
    hand.connect(HOST, PORT)

    try:
        # Init gripper      
        ret = gripper_init(hand)
        if ret:
            hand.disconnect()
            return

        # Rosparam init(for finger_joint)
        rospy.set_param("/gripperSever/gripper_pos", 0)

        print("Start publish...")
        while not rospy.is_shutdown():
            # update joint_state
            gripper_pos = rospy.get_param("/gripperSever/gripper_pos")
            gripper_pos = int(gripper_pos)
            hand.move(gripper_pos, 0, 1)
            (status, position, force) = hand.wait_move_complete()
            # print(gripper_pos)

            joint_state.header.stamp = rospy.Time.now()
            joint_state.name[0] = "finger_joint"
            joint_state.position[0] = gripper_pos*46*3.1415/180/255

            # send the joint state and transform
            joint_pub.publish(joint_state)
            
            rate.sleep()
            signal.signal(signal.SIGINT, handler)
    except:
        print('Ctrl-c pressed')

    hand.disconnect()

if __name__ == '__main__':
    main()
    
    


