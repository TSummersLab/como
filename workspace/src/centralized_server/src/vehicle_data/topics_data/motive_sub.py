import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MotiveSub:
    """
    Subscribes to the Optitrack Pose data via the vrpn client
    """

    def __init__(self, car, pose=None, orientation=None):
        """
        Initializes the MotiveSub for a given vehicle
        -------
        :param car: the namespace of the vehicle of interest
        :param pose: the provided initial pose for the vehicle
        :orientation: the provided initial orientation of the vehicle
        """
        self.data = PoseStamped()

        # Initialize the starting pose
        if pose is None:
            x, y, z = [2.0, 0, 0.3]
        else:
            x, y, z = pose
        self.position = Point(x, y, z)

        # Initialize the starting orientation
        if orientation is None:
            self.orientation = Quaternion()
        else:
            r_x, r_y, r_z = orientation
            q = quaternion_from_euler(r_x, r_y, r_z)
            self.orientation = Quaternion(q[0], q[1], q[2], q[3])

        # Retrieve Motive data for a given vehicle, specified by the provided namespace 'car'
        self.motive_sub = rospy.Subscriber('vrpn_client_node/' + car + '/pose', PoseStamped, self.callback)

    def callback(self, data):
        """
        Callback function for the Motive data from Optitrack
        ----------
        :param data: the data received from the vrpn_client_node for this vehicle (vrpn_client_node/{car}/pose)
        """
        self.data = data
        self.orientation = data.pose.orientation
        self.position = data.pose.position

    def get_pose(self):
        """
        Retrieve the current pose of the vehicle. Currently designed to only provide the 2D state.
        """
        # todo: Update to use the 3D state. Update dependent functions to utilize only the 2D state as required.
        x = self.position.x
        y = self.position.y
        return np.array([x, y])

    def get_orientation(self):
        """
        Retrieve the orientation of the vehicle. Represented in the quaternion form.
        """
        return self.orientation

    def get_yaw(self):
        """
        Retrieve the yaw (orientation around the z-axis) of the vehicle.
        """
        rot_q = self.get_orientation()

        # Convert orientation to Euler angles from the provided quaternion representation
        roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        # Return yaw, represented by theta
        return theta

    def get_time(self):
        """
        Return the timestamp (in nanoseconds) associated with the most recent data message.
        """
        # todo: Update to retrieve time in either seconds or nanoseconds.
        return self.data.header.stamp.to_nsec()

    def get_cur_pose(self):
        """
        Retrieve both the current pose and the current timestamp (in nanoseconds) of the vehicle.
        """
        pos = self.get_pose()
        time = self.get_time()
        return pos, time

