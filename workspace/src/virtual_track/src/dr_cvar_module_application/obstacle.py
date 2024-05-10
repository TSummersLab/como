import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Float64

class Obstacle:
    '''
    Provides all data and functions required for a dynamic obstacle
    '''
    def __init__(self, name):
        self.motive_sub = MotiveSub(name)

class MotiveSub:
    '''
    Subscribes to the Optitrack Pose data via the vrpn client
    '''

    def __init__(self, car):
        self.data = PoseStamped()
        self.position = Point(2.0, 0, 0)
        self.orientation = Quaternion()
        self.motive_sub = rospy.Subscriber('vrpn_client_node/' + car + '/pose', PoseStamped, self.callback)

    def callback(self, data):
        self.data = data
        self.orientation = data.pose.orientation
        self.position = data.pose.position

    def get_pose(self):
        x = self.position.x
        y = self.position.y
        return np.array([x, y])

    def get_orientation(self):
        return self.orientation

    def get_time(self):
        return self.data.header.stamp.to_nsec()

    def get_cur_pose(self):
	pos = self.get_pose()
	time = self.get_time()
	return pos, time

