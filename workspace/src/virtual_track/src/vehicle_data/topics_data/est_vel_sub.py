import rospy
from std_msgs.msg import Float64

class EstVelSub:
    '''
    Subscribes to the estimated velocity, calculated with the Motive data
    '''

    def __init__(self, namespace, velocity=None):
        if velocity is None:
            self.vel = 0.0
        else:
            self.vel = velocity
        
        self.est_vel_sub = rospy.Subscriber(namespace + '/est_vel', Float64, self.callback)
        self.est_vel = 0.0
        self.vel_history = [0.0, 0.0, 0.0, 0.0, 0.0]

    def callback(self, data):
        self.est_vel = data.data
        self.update_velocity()

    def get_velocity(self):
        return self.vel

    def update_velocity(self):
        self.vel_history[:-1] = self.vel_history[1:]
        self.vel_history[-1] = self.est_vel
        self.vel = self.avg_velocity_est()

    def avg_velocity_est(self):
        #TODO: Rename function for clarity
        #factors = [0.1, 0.1, 0.2, 0.3, 0.3]
        #TODO: Add more details on factor selection
        factors = [0.05, 0.075, 0.125, 0.25, 0.5]
        return sum([self.vel_history[i] * factors[i] for i in range(len(factors))])

