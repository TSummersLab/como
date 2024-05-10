import rospy
from std_msgs.msg import Float64
from queue import Queue
from math import sqrt
import random

class EstVelPub:
    '''
    Publish initial velocity estimates from Motive position data
    '''

    def __init__(self, namespace):
        self.est_vel_pub = rospy.Publisher(namespace + '/est_vel', Float64, queue_size=10)
        self.queue_size = 12
        self.pos_queue = Queue(maxsize=self.queue_size)

    def estimate_velocity(self, robot_pos, timestamp, cur_vel_est):
        self.pos_queue.put([robot_pos, timestamp])
        if self.pos_queue.full():
            self.shift_queue(2, 3, 0, cur_vel_est)

    def calculate_vel(self, p1, p2, rate, prev_est):
        del_x = p2[0] - p1[0]
        del_y = p2[1] - p1[1]
        dis = sqrt(del_x ** 2+ del_y ** 2)
        vel_est = dis * rate
        if abs(vel_est - prev_est) > 0.5:
            vel_est = prev_est
        return vel_est

    def trim_outliers(self, vel_est, trim_count):
        #TODO: Modify trim outliers to trim only min/max, based on statistical distribution of the set of estimated velocities
        for i in range(trim_count):
            vel_est.remove(max(vel_est))
            vel_est.remove(min(vel_est))
        return vel_est

    def shift_queue(self, shift_num, trim_count, rand_remove, prev_est):
        # Compute estimated velocities from recent positions
        points = list(self.pos_queue.queue)
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

