import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import tmc_msgs.msg


class SpeechControl(object):
    """ Move arm on button clicks """

    def __init__(self):
        # Create publisher
        self._pub = rospy.Publisher('/talk_request', tmc_msgs.msg.Voice,queue_size=10)

    def speak(self,sentence):
        # fill ROS message
        voice_msg = tmc_msgs.msg.Voice()
        voice_msg.language = 1
        voice_msg.sentence = sentence
        # publish ROS message
        self._pub.publish(voice_msg)