import sys
import copy
from math import pi, tau, dist, fabs, cos
import threading

# ROS
import numpy as np
import rospy
import sensor_msgs.msg
from shape_msgs.msg import SolidPrimitive
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf

# moveit
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import PlanningScene as PlanningScene_msg
from moveit_msgs.msg import CollisionObject
from moveit.core.planning_scene import PlanningScene
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if the values in two lists are within a tolerance of each other.
  For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
  between the identical orientations q and -q is calculated correctly).
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
    x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
    # Euclidean distance
    d = dist((x1, y1, z1), (x0, y0, z0))
    # phi = angle between orientations
    cos_phi_half = fabs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)
    return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

  return True


class MoveGroup(object):
    def __init__(self):
        super(MoveGroup, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        robot_state = RobotState()
        robot_state.joint_state = move_group.get_current_state().joint_state
        # print(move_group.get_current_pose())
        # Publisher and Subscriber
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        state_valid_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)

        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()
        # move_group.allow_replanning(True)

        # Delay 2s to keep the initialization above stable
        rospy.sleep(4)

        self.object_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_name = group_name
        self.group_names = group_names
        self.state_valid_srv = state_valid_srv
        self.robot_state = robot_state

    def go_to_joint_state(self, joint_state):
        move_group = self.move_group

        while True:
            move_group.go(joint_state, wait=True)
            current_joints = move_group.get_current_joint_values()
            if all_close(joint_state, current_joints, 0.01):
                print("stop")
                move_group.stop()
                break
        # while True:
        #     current_joints = move_group.get_current_joint_values()
        #     if all_close(joint_state, current_joints, 0.01):
        #         print("stop")
        #         move_group.stop()
        #         break
        #     rospy.sleep(0.5)

        return

    def go_to_pose_goal(self, pose_goal):
        move_group = self.move_group

        move_group.go(pose_goal.pose, wait=False)

        # # move_group.set_pose_target(pose_goal)
        #
        # ## Now, we call the planner to compute the plan and execute it.
        # plan = move_group.go(pose_goal, wait=True)
        # # Calling `stop()` ensures that there is no residual movement
        # move_group.stop()
        # # It is always good to clear your targets after planning with poses.
        # # Note: there is no equivalent function for clear_joint_value_targets()
        # # move_group.clear_pose_targets()
        #
        # ## END_SUB_TUTORIAL
        #
        # # For testing:
        # # Note that since this section of code will not be included in the tutorials
        # # we use the class variable rather than the copied state variable
        # current_pose = self.move_group.get_current_pose().pose
        # return all_close(pose_goal, current_pose, 0.01)

    def stop(self):
        move_group = self.move_group

        move_group.stop()

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        box_name = self.object_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
          # Test if the box is in attached objects
          attached_objects = scene.get_attached_objects([box_name])
          is_attached = len(attached_objects.keys()) > 0

          is_known = box_name in scene.get_known_object_names()

          if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

          # Sleep so that we give other threads time on the processor
          rospy.sleep(0.1)
          seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_collision(self, obj_name, positions, timeout=4):
        scene = self.scene
        self.object_name = obj_name

        # Create object
        obj = SolidPrimitive()
        obj.type = obj.SPHERE
        obj.dimensions = [0.1]

        obj2 = SolidPrimitive()
        obj2.type = obj.SPHERE
        obj2.dimensions = [0.15]

        # Add object as collision
        obj_msg = CollisionObject()

        for idx, pos in enumerate(positions):
            # Create object position
            obj_pose = geometry_msgs.msg.Pose()
            obj_pose.orientation.w = 1.0
            obj_pose.position.x = pos.z / 1000.0
            obj_pose.position.y = pos.x / -1000.0
            obj_pose.position.z = pos.y / 1000

            if idx in [7, 8, 12, 13]:
                obj_msg.primitives.append(obj2)
            else:
                obj_msg.primitives.append(obj)
            obj_msg.primitive_poses.append(obj_pose)

        obj_msg.header.frame_id = "nuitrack_link"
        obj_msg.id = obj_name
        obj_msg.operation = CollisionObject.ADD

        # Add collision info to planning scene
        scene.add_object(obj_msg)
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.object_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        grasping_group = 'hand'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.object_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.object_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def get_joint_state(self):
        move_group = self.move_group
        return move_group.get_current_joint_values()

    def get_current_pose(self):
        move_group = self.move_group
        return move_group.get_current_pose()

    # private:
    def _skl_callback(self, msg):
        if msg.skeletons:
            skl_data = msg.skeletons[0]
            self.add_collision("hand", skl_data.joint_pos)

    def _get_state_validity(self, constraints=None):
        '''
        RobotState robot_state
        string group_name
        Constraints constraints

        ---

        bool valid
        ContactInformation[] contacts
        CostSource[] cost_sources
        ConstraintEvalResult[] constraint_result
        '''

        self.robot_state.joint_state = self.move_group.get_current_state().joint_state

        gsvr = GetStateValidityRequest()
        gsvr.robot_state = self.robot_state
        gsvr.group_name = self.group_name
        if constraints is not None:
            gsvr.constraints = constraints
        result = self.state_valid_srv.call(gsvr)
        return result


def callback(msg):
    for marker in msg.markers:
        if marker.ns == 'mouse':
            data = marker.pose
            broadcaster = tf.TransformBroadcaster()
            broadcaster.sendTransform((data.position.x, data.position.y, data.position.z),
                                      (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w),
                                      rospy.Time().now(), '/mouse', '/cam_1_color_optical_frame')
            break


def task():
    rospy.init_node('collision_avoid', anonymous=True)
    move = MoveGroup()
    sub = rospy.Subscriber("/cam_1/object_detection/markers", MarkerArray, callback, queue_size=1)
    listener = tf.TransformListener()

    listener.waitForTransform('/base_link', '/mouse', rospy.Time(), rospy.Duration(10))
    rec_x, rec_y = 0, 0
    while True:
        move_flag = False
        (trans, rot) = listener.lookupTransform('/base_link', '/mouse', rospy.Time(0))
        print(trans)
        if (abs(rec_x-trans[0])+rec_y-trans[1]) >= 0.05 and not move_flag:
            pose_goal = move.get_current_pose()
            pose_goal.pose.position.x = trans[0]
            pose_goal.pose.position.y = trans[1]
            move.go_to_pose_goal(pose_goal)
            move_flag = True
            rec_x, rec_y = trans[0], trans[1]
        rospy.sleep(1.0)
        if move_flag:
            current_pose = move.get_current_pose()
            if all_close(pose_goal, current_pose, 0.01):
                print("stop")
                move_flag = False
                move.stop()


def main():
    try:
        task()
        rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
