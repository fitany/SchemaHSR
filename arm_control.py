import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg
from control_msgs.msg import JointTrajectoryControllerState

import actionlib
from actionlib_msgs.msg import GoalStatus
import control_msgs.msg


class ArmControl(object):
    """ Move arm on button clicks """

    def __init__(self):

        # Subscribe arm state data from HSR
        self.arm_state_sub = rospy.Subscriber('/hsrb/arm_trajectory_controller/state', JointTrajectoryControllerState,
                                           self.receive_state)
        # Wait until connection
        rospy.wait_for_message('/hsrb/arm_trajectory_controller/state', JointTrajectoryControllerState, timeout=100.0)
        # Store positions received from topic
        self.actual_positions = [0, 0, 0, 0, 0]
        self.desired_positions = [0, 0, 0, 0, 0]

        # Create publisher
        self._pub = rospy.Publisher('/hsrb/arm_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory,
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
                if c.name == 'arm_trajectory_controller' and c.state == 'running':
                    running = True

        # initialize action client
        self.cli = actionlib.SimpleActionClient(
            '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
            control_msgs.msg.FollowJointTrajectoryAction)
        # wait for the action server to establish connection
        self.cli.wait_for_server()

    def auto_grasp_floor_object(self,height=0.0):
        # fill ROS message

        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                            "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [height, 0.0, 0.0,-1.30, 0.0]
        p.velocities = [0, 0, 0, 0, 0]
        p.time_from_start = rospy.Time(3)
        traj.points = [p]
        goal.trajectory = traj
        # send message to the action server
        self.cli.send_goal(goal)
        # wait for the action server to complete the order
        self.cli.wait_for_result()
        action_state = self.cli.get_state()
        if action_state == GoalStatus.SUCCEEDED:
            print 'Raise Arm Succeeded.'
        else:
            print 'Raise Arm Failed.'

        p = trajectory_msgs.msg.JointTrajectoryPoint()
        if height < .5:
            p.positions = [height, -1.8, 0.0, -1.30, 0.0]
        else:
            p.positions = [height, -1.6, 0.0, -1.30, 0.0]
            if height > .7 and height < .73:
                p.positions = [height, -1.6, 0.0, -1.30, 0.0]
        p.velocities = [0, 0, 0, 0, 0]
        p.time_from_start = rospy.Time(3)
        traj.points = [p]
        goal.trajectory = traj
        # send message to the action server
        self.cli.send_goal(goal)
        # wait for the action server to complete the order
        self.cli.wait_for_result()
        # print result of arm action
        action_state = self.cli.get_state()
        if action_state == GoalStatus.SUCCEEDED:
            print 'Arm Action Succeeded.'
        else:
            print 'Arm Action Failed.'

    def clicked_arm_raise(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0.67, -0.4, 0.0, 0.0, 0.0]
        p.time_from_start = rospy.Time(5)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_arm_lower(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0.0, -0.0, 0.0, -1.5, 0.0]
        p.time_from_start = rospy.Time(5)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_arm_neutral(self):
        '''
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0.0, -0.0, -1.5, -1.5, 0.0]
        p.time_from_start = rospy.Time(5)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)
        '''
        # initialize action client
        cli = actionlib.SimpleActionClient(
            '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
            control_msgs.msg.FollowJointTrajectoryAction)
        # wait for the action server to establish connection
        cli.wait_for_server()
        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy(
            '/hsrb/controller_manager/list_controllers',
            controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'arm_trajectory_controller' and c.state == 'running':
                    running = True
        # fill ROS message
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                            "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0.0, -0.0, -1.3, -1.5, 0.0]
        p.velocities = [0, 0, 0, 0, 0]
        p.time_from_start = rospy.Time(3)
        traj.points = [p]
        goal.trajectory = traj
        # send message to the action server
        cli.send_goal(goal)
        # wait for the action server to complete the order
        cli.wait_for_result()
        # print result of arm action
        action_state = cli.get_state()
        if action_state == GoalStatus.SUCCEEDED:
            print 'Arm Action Succeeded.'
        else:
            print 'Arm Action Failed.'

    def clicked_at_target(self, target_z):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint",
                            "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [target_z - 0.6, -0.3, 0.0, -1.5, 0.0]
        p.time_from_start = rospy.Time(5)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_raise_trunk(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint",
                            "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0]+.1, -0.25, self.actual_positions[2], self.actual_positions[3],
                       self.actual_positions[4]]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_lower_trunk(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint",
                            "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0]-.1, -0.25, self.actual_positions[2], self.actual_positions[3],
                       self.actual_positions[4]]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def receive_state(self, data):
        self.actual_positions = data.actual.positions
        self.desired_positions = data.desired.positions

    def shutdown(self):
        self._pub.publish(trajectory_msgs.msg.JointTrajectory())
