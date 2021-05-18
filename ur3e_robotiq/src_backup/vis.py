from __future__ import print_function

import sys
import copy

import rospy
import moveit_commander
import moveit_msgs.msg

import pyrealsense2 as rs
import numpy as np
import cv2

color_dist = {'red': {'Lower': np.array([0, 119, 29]), 'Upper': np.array([179, 255, 185])},
              }

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale


# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

'''-----------------------------------------------------------------------------'''
# Initialize instance
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

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        # if depth_colormap_dim != color_colormap_dim:
        #     resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
        #     images = np.hstack((resized_color_image, depth_colormap))
        # else:
        #     images = np.hstack((color_image, depth_colormap))

        gs_frame = cv2.GaussianBlur(color_image, (5, 5), 0)  # 高斯模糊
        hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)  # 转化成HSV图像
        erode_hsv = cv2.erode(hsv, None, iterations=2)
        inRange_hsv = cv2.inRange(erode_hsv, color_dist['red']['Lower'], color_dist['red']['Upper'])
        inRange_hsv[:150, :] = 0

        cnts = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        c = max(cnts, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        cv2.drawContours(color_image, [np.int0(box)], -1, (0, 255, 255), 2)

        # Get x, y, depth from the object
        x = sum(box[:, 1])/4
        y = sum(box[:, 0])/4
        depth = 0
        for point in c:
            depth += depth_image[point[0][1], point[0][0]]
        depth /= c.shape[0]
        # print("====>>>> Current depth is:", depth, "x is:", x, "y is:", y)

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            scale = 1.0
            waypoints = []

            real_x = -(300+(220-y)/1.05)/1000
            real_y = -(130-(depth-450))/1000
            print("====>>>> Current depth is:", depth, "x is:", x, "y is:", y)
            print(move_group.get_current_pose())
            print("====>>>>", "x is:", real_x, "y is:", real_y)
            wpose = move_group.get_current_pose().pose
            wpose.position.x = real_x  # First move up (z)
            wpose.position.y = real_y  # and sideways (y)
            waypoints.append(copy.deepcopy(wpose))

            # We want the Cartesian path to be interpolated at a resolution of 1 cm
            # which is why we will specify 0.01 as the eef_step in Cartesian
            # translation.  We will disable the jump threshold by setting it to 0.0,
            # ignoring the check for infeasible jumps in joint space, which is sufficient
            # for this tutorial.
            (plan, fraction) = move_group.compute_cartesian_path(
                waypoints,  # waypoints to follow
                0.01,  # eef_step
                0.0)  # jump_threshold

            # Note: We are just planning, not asking move_group to actually move the robot yet:
            # return plan, fraction

            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            # Publish
            display_trajectory_publisher.publish(display_trajectory)
            move_group.execute(plan, wait=True)

finally:

    # Stop streaming
    pipeline.stop()
