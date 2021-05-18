from __future__ import print_function
from six.moves import input

import sys
import copy
from math import pi, dist, fabs, cos, tau

import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

import time
import signal
from RobotiqHand import RobotiqHand

position = [[-0.10376006761659795, -1.4604805272868653, 1.7120140234576624, -1.8408385715880335, -1.5818913618670862, -0.10119229951967412],# pos1
            [-0.1037362257586878, -1.2928179067424317, 1.966555420552389, -2.2628704510130824, -1.5843380133258265, -0.1004870573626917],# pos1-d
            [-2.48826772371401, -1.0086077016643067, 1.5154560248004358, -2.076402326623434, -1.545046631489889, -2.484631363545553],# pos1-down
            [-0.7767108122455042, -1.4034460347941895, 1.62944204012026, -1.820939680139059, -1.5675109068499964, -0.7739060560809534],# pos2
            [-0.7766030470477503, -1.2357590955546875, 1.8955076376544397, -2.2547246418394984, -1.570113484059469, -0.7731393019305628],# pos2-d
            [-2.48826772371401, -1.0594738286784668, 1.4519642035113733, -1.9619518719115199, -1.5443270842181605, -2.4848716894732874],# pos2-dd
            [-1.0771129767047327, -0.8735049527934571, 1.0427072683917444, -1.7633797130980433, -1.560110870991842, -1.0737898985492151],# pos3
            [-1.0838654677020472, -0.7882493299296875, 1.1449039618121546, -1.9507481060423792, -1.5611303488360804, -1.0799711386310022],# pos3-d
            [-2.4882078806506556, -1.1121552747539063, 1.3280742804156702, -1.7854439220824183, -1.5434039274798792, -2.4853885809527796],# pos3-dd
            [-1.456583325062887, -1.2193835538676758, 1.668490235005514, -2.039511343041891, -1.5539820829974573, -1.4530704657184046],# pos4
            [-1.448374096547262, -1.1096347135356446, 1.723166290913717, -2.204036851922506, -1.555157486592428, -1.4446314016925257],# pos4-d
            [-2.4883039633380335, -1.1141742032817383, 1.3203628698932093, -1.775698801080221, -1.5432599226581019, -2.485352341328756], #pos4-dd
            ]
position2 = [-2.4883039633380335, -1.1349614423564454, 1.1539743582354944, -1.5884491405882777, -1.5422404448138636, -2.4860380331622522]
### Initialize instance ###
# Initialize moveit_commander and rospy nodes
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('interface_test', anonymous=True)

# Instantiate a RobotCommander object
# Provides information such as kinematic model and the current joint states
robot = moveit_commander.RobotCommander()

# Instantiate a PlanningSceneInterface object
# Provides a remote interface for getting, setting, and updating 
# the robot’s internal understanding of the surrounding world
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a MoveGroupCommander object
# This object is an interface to a planning group (group of joints)
# Need to be changed with different robots
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# Gripper Init
HOST = "192.168.0.111"
PORT = 54321
cont = True

def handler(signal, frame):
    global cont
    cont = False
signal.signal(signal.SIGINT, handler)

print('test_robotiq start')
hand = RobotiqHand()
hand.connect(HOST, PORT)

print('activate: start')
hand.reset()
hand.activate()
result = hand.wait_activate_complete()
print('activate: result = 0x{:02x}'.format(result))
if result != 0x31:
    hand.disconnect()
    # return
print('adjust: start')
hand.adjust()
print('adjust: finish')

### Getting basic information ###
# We can get the name of the reference frame for this robot:
# planning_frame = move_group.get_planning_frame()
# print("============ Planning frame: %s" % planning_frame)
#
# # We can also print the name of the end-effector link for this group:
# eef_link = move_group.get_end_effector_link()
# print("============ End effector link: %s" % eef_link)
#
# # We can get a list of all the groups in the robot:
# group_names = robot.get_group_names()
# print("============ Available Planning Groups:", group_names)

# Sometimes for debugging it is useful to print the entire state of the
# robot:
#hand.move(0, 255, 0)
#hand.move(255, 0, 1)
print("============ Printing robot state")
print(robot.get_current_state())
print(move_group.get_current_pose())
print("")
# [1.276915060650208, 1.1232200736847435, 0.4610743586339171,1]
# xyz="-1.06523 -0.510126 0.22145" rpy="0.0559442 0.0732148 -0.286719" -->
# # Fixed joint positions used
# for i in range(4):
#     pos = position[i*3:((i+1)*3)]
#     for j in range(6):
#         if j == 0 or j == 1:
#             p = pos[j]
#         elif j == 2:
#             p = pos[0]
#         elif j == 3 or j == 5:
#             p = position2
#         elif j == 4:
#             p = pos[2]
#         move_group.go(p, wait=True)
#         move_group.stop()
#         if j==1:
#             print('close slow')
#             hand.move(255, 0, 1)
#             (status, h_position, force) = hand.wait_move_complete()
#             position_mm = hand.get_position_mm(h_position)
#             force_mA = hand.get_force_mA(force)
#         if j==4:
#             print('open fast')
#             hand.move(0, 255, 0)
#             (status, h_position, force) = hand.wait_move_complete()
#             position_mm = hand.get_position_mm(h_position)
#             force_mA = hand.get_force_mA(force)
#             print('position = {:.1f}mm, force = {:.1f}mA '.format(position_mm, force_mA))

### Planning to a Pose Goal ###
# We can plan a motion for this group to a desired pose for the end-effector
pose_goal = move_group.get_current_pose().pose
pose_goal.position.x = -0.08  # First move up (z)
# pose_goal.position.y = -0.3
pose_goal.position.z = 0.30

# move_group.set_pose_target(pose_goal, end_effector_link='tool')
#
plan = move_group.plan(pose_goal)

display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan[1])
# Publish
display_trajectory_publisher.publish(display_trajectory)

move_group.go(wait=True)
# # Calling `stop()` ensures that there is no residual movement
move_group.stop()
# # It is always good to clear your targets after planning with poses.
# # Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()

#
# print(move_group.get_planning_time())
# scale = 1.0
#
# waypoints = []
# #
# wpose = move_group.get_current_pose().pose
# wpose.position.x = 0.08  # First move up (z)
# wpose.position.y = -0.3
# wpose.position.z = 0.34
# # wpose.position.y += -0.10  # and sideways (y)
# waypoints.append(copy.deepcopy(wpose))
#
# # We want the Cartesian path to be interpolated at a resolution of 1 cm
# # which is why we will specify 0.01 as the eef_step in Cartesian
# # translation.  We will disable the jump threshold by setting it to 0.0,
# # ignoring the check for infeasible jumps in joint space, which is sufficient
# # for this tutorial.
# (plan, fraction) = move_group.compute_cartesian_path(
#                                    waypoints,   # waypoints to follow
#                                    0.01,        # eef_step
#                                    0)         # jump_threshold
#
# # Note: We are just planning, not asking move_group to actually move the robot yet:
# # return plan, fraction
#
# display_trajectory = moveit_msgs.msg.DisplayTrajectory()
# display_trajectory.trajectory_start = robot.get_current_state()
# display_trajectory.trajectory.append(plan)
# # Publish
# display_trajectory_publisher.publish(display_trajectory)
# move_group.execute(plan, wait=True)