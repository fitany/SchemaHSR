import rospy
import controller_manager_msgs.srv
import trajectory_msgs.msg

import actionlib
import tmc_control_msgs.msg
import control_msgs.msg
from actionlib_msgs.msg import GoalStatus

class GripperControl(object):
    """ Move gripper on button clicks """

    def __init__(self):

        # Create publisher
        self._pub = rospy.Publisher('/hsrb/gripper_controller/command', trajectory_msgs.msg.JointTrajectory,
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
                if c.name == 'gripper_controller' and c.state == 'running':
                    running = True

        # initialize action client
        self.cli_open = actionlib.SimpleActionClient(
            '/hsrb/gripper_controller/follow_joint_trajectory',
            control_msgs.msg.FollowJointTrajectoryAction)
        # wait for the action server to establish connection
        self.cli_open.wait_for_server()

        # initialize action client
        self.cli_close = actionlib.SimpleActionClient(
            '/hsrb/gripper_controller/grasp',
            tmc_control_msgs.msg.GripperApplyEffortAction)
        # wait for the action server to establish connection
        self.cli_close.wait_for_server()

    def clicked_hand_gripper(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["hand_motor_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [-0.5]
        p.velocities = [0.1]
        p.effort = [0.1]
        p.time_from_start = rospy.Time(3)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_gripper_open(self):
        '''
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["hand_motor_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [1.2]
        p.velocities = [0.1]
        p.effort = [0.1]
        p.time_from_start = rospy.Time(3)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)
        '''
        # fill ROS message
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["hand_motor_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [1.2]
        p.velocities = [0]
        p.effort = [0.1]
        p.time_from_start = rospy.Time(3)
        traj.points = [p]
        goal.trajectory = traj
        # send message to the action server
        self.cli_open.send_goal(goal)
        # wait for the action server to complete the order
        self.cli_open.wait_for_result()

    def clicked_gripper_close(self):
        '''
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["hand_motor_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [-1.0]
        p.velocities = [0.1]
        p.effort = [0.1]
        p.time_from_start = rospy.Time(3)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)
        '''
        # fill ROS message
        goal = tmc_control_msgs.msg.GripperApplyEffortGoal()
        goal.effort = -.01
        # send message to the action server
        self.cli_close.send_goal(goal)
        # wait for the action server to complete the order
        self.cli_close.wait_for_result()