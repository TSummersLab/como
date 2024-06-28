import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MotiveSub:
    '''
    Subscribes to the Optitrack Pose data via the vrpn client
    '''

    def __init__(self, car, pose=None, orientation=None):
        self.data = PoseStamped()

        if pose is None:
            x, y, z = [2.0, 0, 0.3]
        else:
            x, y, z = pose
        self.position = Point(x, y, z)

        if orientation is None:
            self.orientation = Quaternion()
        else:
            r_x, r_y, r_z = orientation
            q = quaternion_from_euler(r_x, r_y, r_z)
            self.orientation = Quaternion(q[0], q[1], q[2], q[3])

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

    def get_yaw(self):
        rot_q = self.get_orientation()
        roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        return theta

    def get_time(self):
        return self.data.header.stamp.to_nsec()

    def get_cur_pose(self):
        pos = self.get_pose()
        time = self.get_time()
        return pos, time

