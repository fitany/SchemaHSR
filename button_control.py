# Author: H J Kashyap, T Hwu
import rospy
import controller_manager_msgs.srv
import geometry_msgs.msg
import trajectory_msgs.msg
from control_msgs.msg import JointTrajectoryControllerState
import control_msgs.msg
import yaml
import time
import actionlib
import thread

class BaseTrajectoryControl(object):
    """ Move base to a position """

    def __init__(self):
        # Subscribe color image data from HSR
        self._base_state_sub = rospy.Subscriber('/hsrb/omni_base_controller/state', JointTrajectoryControllerState, self.receive_state)
        # Wait until connection
        rospy.wait_for_message('/hsrb/omni_base_controller/state', JointTrajectoryControllerState, timeout=100.0)
        # Store positions received from topic
        self.actual_positions = (0, 0, 0)
        self.desired_positions = (0, 0, 0)
        self.actual_vel = (0, 0, 0)
        self.desired_vel = (0, 0, 0)
        with open("param.yaml", "r") as f:
            self.param = yaml.load(f)

        # initialize ROS publisher
        self._pub = rospy.Publisher('/hsrb/omni_base_controller/command', trajectory_msgs.msg.JointTrajectory,
                                    queue_size=10)

        # wait to establish connection between the controller
        while self._pub.get_num_connections() == 0:
            rospy.sleep(0.1)

        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                                              controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'omni_base_controller' and c.state == 'running':
                    running = True

    def clicked_left_90(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["odom_x", "odom_y", "odom_t"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1], self.actual_positions[2] + 1.5708]
        p.velocities = [0.0, 0.0, 0.3]
        p.time_from_start = rospy.Time(self.param["rotate_90"]["time_to_rotate"])
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_right_90(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["odom_x", "odom_y", "odom_t"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1], self.actual_positions[2] - 1.5708]
        p.velocities = [0.0, 0.0, 0.3]
        p.time_from_start = rospy.Time(self.param["rotate_90"]["time_to_rotate"])
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_left_45(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["odom_x", "odom_y", "odom_t"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1], self.actual_positions[2] + 0.775]
        p.velocities = [0.0, 0.0, 0.3]
        p.time_from_start = rospy.Time(self.param["rotate_45"]["time_to_rotate"])
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_right_45(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["odom_x", "odom_y", "odom_t"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1], self.actual_positions[2] - 0.775]
        p.velocities = [0.0, 0.0, 0.3]
        p.time_from_start = rospy.Time(self.param["rotate_45"]["time_to_rotate"])
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def receive_state(self, data):
        self.actual_positions = data.actual.positions
        self.desired_positions = data.desired.positions
        self.actual_vel = data.actual.velocities
        self.desired_vel = data.desired.velocities

    def shutdown(self):
        self._pub.publish(trajectory_msgs.msg.JointTrajectory())


class BaseControl(object):
    """ Move base with a velocity on button clicks """

    def __init__(self):
        # Create publisher
        self._pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)
        # Wait to establish connection between the controller
        while self._pub.get_num_connections() == 0:
            rospy.sleep(0.1)
        # Make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                                              controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'omni_base_controller' and c.state == 'running':
                    running = True

    def clicked_forward(self):
        # fill ROS message
        tw = geometry_msgs.msg.Twist()
        tw.linear.x =0.2
        # publish ROS message
        self._pub.publish(tw)

    def clicked_backward(self):
         # fill ROS message
         tw = geometry_msgs.msg.Twist()
         tw.linear.x = -0.2 # slow to discourage use
         # publish ROS message
         self._pub.publish(tw)

    def clicked_left(self):
        # fill ROS message
        tw = geometry_msgs.msg.Twist()
        tw.angular.z = 0.2
        # publish ROS message
        self._pub.publish(tw)

    def clicked_right(self):
        # fill ROS message
        tw = geometry_msgs.msg.Twist()
        tw.angular.z = -0.2
        # publish ROS message
        self._pub.publish(tw)

    def released(self):
        # fill ROS message
        tw = geometry_msgs.msg.Twist()
        # publish ROS message
        self._pub.publish(tw)

    def shutdown(self):
        self._pub.publish(geometry_msgs.msg.Twist())

class HeadControl(object):
    """ Move head on button clicks """

    def __init__(self):
        # Subscribe color image data from HSR
        self._image_sub = rospy.Subscriber('/hsrb/head_trajectory_controller/state', JointTrajectoryControllerState, self.receive_state)
        # Wait until connection
        rospy.wait_for_message('/hsrb/head_trajectory_controller/state', JointTrajectoryControllerState, timeout=100.0)
        # Store positions received from topic
        self.actual_positions = (0, 0)
        self.desired_positions = (0, 0)

        # Boolean for direction of head sweep
        self.sweep_left = True

        # Create publisher
        self._pub = rospy.Publisher('/hsrb/head_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory,
                                    queue_size=10)
        # Wait to establish connection between the controller
        while self._pub.get_num_connections() == 0:
            rospy.sleep(0.1)
        # Make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                                              controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'head_trajectory_controller' and c.state == 'running':
                    running = True

        # initialize head action client
        self.cli_head = actionlib.SimpleActionClient(
            '/hsrb/head_trajectory_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
        # wait for the action server to establish connection
        self.cli_head.wait_for_server()

    def head_sweep(self,pan_pos):
        # fill ROS message
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [pan_pos, 0.0]
        p.velocities = [0, 0]
        p.time_from_start = rospy.Time(3)
        traj.points = [p]
        goal.trajectory = traj
        # send message to the action server
        self.cli_head.send_goal(goal)
        # wait for the action server to complete the order
        self.cli_head.wait_for_result()
        time.sleep(1)

    def clicked_up(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[1], self.actual_positions[0]+.1]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_down(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[1], self.actual_positions[0]-.1]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_left(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[1]+.1, self.actual_positions[0]]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_right(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[1]-.1, self.actual_positions[0]]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_home(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0,0]
        p.time_from_start = rospy.Time(1)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_left_45(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[1] + 0.775, self.actual_positions[0]]
        p.time_from_start = rospy.Time(2.5)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_right_45(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[1] - 0.775, self.actual_positions[0]]
        p.time_from_start = rospy.Time(2.5)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def receive_state(self, data):
        self.actual_positions = data.actual.positions
        self.desired_positions = data.desired.positions

    def shutdown(self):
        self._pub.publish(trajectory_msgs.msg.JointTrajectory())


# class GripperControl(object):
#     """ Move gripper on button clicks """
#
#     def __init__(self):
#
#         # Create publisher
#         self._pub = rospy.Publisher('/hsrb/gripper_controller/command', trajectory_msgs.msg.JointTrajectory,
#                                     queue_size=10)
#         # Wait to establish connection between the controller
#         while self._pub.get_num_connections() == 0:
#             rospy.sleep(0.1)
#         # Make sure the controller is running
#         rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
#         list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
#                                               controller_manager_msgs.srv.ListControllers)
#         running = False
#         while running is False:
#             rospy.sleep(0.1)
#             for c in list_controllers().controller:
#                 if c.name == 'gripper_controller' and c.state == 'running':
#                     running = True
#
#     def clicked_hand_gripper(self):
#         # fill ROS message
#         traj = trajectory_msgs.msg.JointTrajectory()
#         traj.joint_names = ["hand_motor_joint"]
#         p = trajectory_msgs.msg.JointTrajectoryPoint()
#         p.positions = [-0.5]
#         p.velocities = [0.1]
#         p.effort = [0.1]
#         p.time_from_start = rospy.Time(3)
#         traj.points = [p]
#         # publish ROS message
#         self._pub.publish(traj)
#
#     def clicked_gripper_open(self):
#         # fill ROS message
#         traj = trajectory_msgs.msg.JointTrajectory()
#         traj.joint_names = ["hand_motor_joint"]
#         p = trajectory_msgs.msg.JointTrajectoryPoint()
#         p.positions = [1.2]
#         p.velocities = [0.1]
#         p.effort = [0.1]
#         p.time_from_start = rospy.Time(3)
#         traj.points = [p]
#         # publish ROS message
#         self._pub.publish(traj)
#
#     def clicked_gripper_close(self):
#         # fill ROS message
#         traj = trajectory_msgs.msg.JointTrajectory()
#         traj.joint_names = ["hand_motor_joint"]
#         p = trajectory_msgs.msg.JointTrajectoryPoint()
#         p.positions = [-1.0]
#         p.velocities = [0.1]
#         p.effort = [0.1]
#         p.time_from_start = rospy.Time(3)
#         traj.points = [p]
#         # publish ROS message
#         self._pub.publish(traj)
