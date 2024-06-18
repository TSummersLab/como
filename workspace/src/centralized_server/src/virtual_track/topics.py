import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Float64
from barc.msg import Encoder

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

class EstVelSub:
    '''
    Subscribes to the estimated velocity, calculated with the Motive data
    '''

    def __init__(self):
        self.vel = 0.0
        self.est_vel_sub = rospy.Subscriber('/est_vel', Float64, self.callback)
	self.vel_history = [0.0, 0.0, 0.0, 0.0, 0.0]

    def callback(self, data):
        self.vel = data.data
	self.update_velocity()

    def get_velocity(self):
	return self.vel

    def update_velocity(self):
	self.vel_history[:-1] = self.vel_history[1:]
	self.vel_history[-1] = self.vel

    def avg_velocity_est(self):
	#factors = [0.1, 0.1, 0.2, 0.3, 0.3]
	factors = [0.05, 0.075, 0.125, 0.25, 0.5]
	return sum([self.vel_history[i] * factors[i] for i in range(len(factors))])
	#return sum(self.vel_history)/len(self.vel_history)

class ErrorSub:
    '''
    Subscribes to error data, such as error in lateral deviation
    '''
    
    def __init__(self):
	self.lateral_deviation = 0.0
	self.error_data_sub = rospy.Subscriber('/data/track_deviation', Float64, self.callback)

    def callback(self, data):
	self.lateral_deviation = data.data

    def get_lateral_deviation(self):
	return self.lateral_deviation

class EncoderSub:
    '''
    Subscribes to the velocity estimation by the encoders.
    A function of the barc package.
    '''

    def __init__(self):
	self.vel_est = 0.0
	self.encoder_estimates = [0.0, 0.0, 0.0, 0.0]
	self.vel_est_sub = rospy.Subscriber('/vel_est', Encoder, self.callback)

    def callback(self, data):
	self.encoder_estimates = [data.FL, data.FR, data.BL, data.BR]
	self.vel_est = self.estimate_avg_velocity()

    def get_estimated_velocity(self):
	return self.vel_est

    def estimate_avg_velocity(self):
	return sum(self.encoder_estimates)/4
	
