import rospy
from std_msgs.msg import Float64
from queue import Queue
from math import sqrt
import random

class EstVelPub:
    """
    Publish initial velocity estimates from MotiveSub (Optitrack Motive) position data
    """

    def __init__(self, namespace):
        self.est_vel_pub = rospy.Publisher(namespace + '/est_vel', Float64, queue_size=10)
        self.queue_size = 12
        self.pos_queue = Queue(maxsize=self.queue_size)

    def estimate_velocity(self, robot_pos, timestamp, cur_vel_est):
        """Trigger the first velocity estimation filter upon reaching max capacity in the position queue"""
        self.pos_queue.put([robot_pos, timestamp]) # Update the position queue with the most recent position data
        if self.pos_queue.full():
            # If the position queue is full, generate a velocity estimate and update the queue
            self.shift_queue(2, 3, 0, cur_vel_est)

    @staticmethod
    def calculate_vel(p1, p2, rate, prev_est):
        """
        Compute the absolute velocity
        ------
        :param p1: position (x, y) data of the first data point
        :param p2: position (x, y) data of the second data point
        :param rate: the rate at which the the vehicle is moving from p1 to p2
        :param prev_est: the current velocity estimate
        """

        del_x = p2[0] - p1[0]
        del_y = p2[1] - p1[1]
        dis = sqrt(del_x ** 2+ del_y ** 2)
        vel_est = dis * rate

        # If the new estimated velocity exceeds the previous estimate by a given margin, overwrite it with the
        # previous estimate. This serves to limit any velocity estimate outliers. This is acceptable due to the fast
        # update rate and computation of this dual-layer filter.
        if abs(vel_est - prev_est) > 0.5:
            vel_est = prev_est
        return vel_est

    @staticmethod
    def trim_outliers(vel_est, trim_count):
        """
        Trim the outliers from the set of velocity estimates
        -------
        :param vel_est: the list of velocity estimates (size is 1 less than that of the position queue)
        :param trim_count: the number of outliers to trim (on both sides)
        """

        # todo: Modify trim outliers to appropriately trim velocities based on the statistical distribution of the set

        for i in range(trim_count):
            vel_est.remove(max(vel_est)) # Remove the largest velocity estimate from the list
            vel_est.remove(min(vel_est)) # Remove the smallest velocity estimate from the list
        return vel_est

    def shift_queue(self, shift_num, trim_count, rand_remove, prev_est):
        """
        Generate a set of initial velocity estimates and update/shift the position queue for new/incoming data
        ------
        :param shift_num: the number of entries by which to shift the queue upon trigger
        :param trim_count: the number of velocity estimates to trim (on both sides)
        :param rand_remove: the number of random velocities to remove at random
        :param prev_est: the current velocity estimate
        """

        points = list(self.pos_queue.queue) # Reformat the queue as a list of data points

        # Compute an initial set of velocity estimates between each consecutive pair of position points
        vel_est = []
        for i in range(self.queue_size - 1):
            del_time = (points[i + 1][1] - points[i][1])/(10.0 ** 9)
            if del_time == 0:
                del_time = 10000000
            vel_est.append(self.calculate_vel(points[i][0], points[i + 1][0], 1/del_time, prev_est))

        # Trim outliers in set of estimated velocities
        vel_est = self.trim_outliers(vel_est, trim_count)
        
        # Remove estimated velocities at random
        for i in range(rand_remove):
            vel_est.remove(random.choice(vel_est))

        # Average estimated velocities
        avg_vel_est = sum(vel_est)/len(vel_est)
        
        # Publish estimated velocity
        self.est_vel_pub.publish(avg_vel_est)
        
        # Shift queue by 'shift_num'
        for i in range(shift_num):
            robot_pos = self.pos_queue.get()

