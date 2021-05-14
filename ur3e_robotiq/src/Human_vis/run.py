import pyrealsense2 as rs
import numpy as np
import cv2
import time
import process
import math

import sys
import rospy
import moveit_commander
import moveit_msgs.msg


def move(pos):
    scale = 0.5
    pose_goal = move_group.get_current_pose().pose
    pose_goal.position.x = min(max(pos[0]+0.3, -0.3), 0.3)
    # pose_goal.position.y += max((pos[1]-pose_goal.position.y)*scale, -0.45)
    # pose_goal.position.z += min((pos[2]-pose_goal.position.z)*scale, 0.35)

    move_group.set_pose_target(pose_goal, end_effector_link='tool')
    #
    plan = move_group.plan(pose_goal)

    if plan[0]:
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
        # rospy.sleep(5)
    else:
        print("****************************\n")
        print(pos)
        print(move_group.get_current_pose().pose)
        print(pose_goal)
        print("****************************\n")


x_scale = math.tan(35/180*3.1415926)
y_scale = math.tan(21.5/180*3.1415926)
r = [[-0.5479, 0.0499, 0.8351, -1.42422],
     [-0.8362, -8.97e-4, -0.5485, -0.133581],
     [-0.0266, -0.9988, 0.0422, 0.0864915],
     [0, 0, 0, 1]]

### Robot init
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('interface_test', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

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
print("Depth Scale is: ", depth_scale)

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

try:
    ske = process.Skeleton()
    s_time = time.time()
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

        c_time = time.time()
        if c_time - s_time > 2:
            s_time = c_time
            cv2.imwrite("temp.jpg", color_image)
            img, flag = ske.skeleton_point(color_image)

            if flag:
                px, py, score = ske.get_center()
                if score > 0.5:
                    deep = np.mean(np.mean(depth_image[py - 20:py + 20, px - 20:px + 20])) * depth_scale
                    x_len = deep * x_scale
                    y_len = deep * y_scale
                    real_x = (px - 640) * x_len / 640
                    real_y = (py - 360) * y_len / 360
                    p = [real_x, -real_y, deep, 1]
                    trans_p = np.matmul(r, np.transpose(p))
                    pos_in = [trans_p[0], trans_p[1], trans_p[2]]
                    move(pos_in)
                    print(p, '\n', trans_p)

            cv2.imshow('ske', img)
        cv2.imshow('video', color_image)
        cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()
