import rospy
from std_msgs.msg import Float64


class EstVelSub:
    """
    Subscribes to the estimated velocity, calculated with the Motive data
    """

    def __init__(self, namespace, velocity=None):
        # self.vel holds the final estimated velocity by this class
        if velocity is None:
            self.vel = 0.0
        else:
            self.vel = velocity

        self.est_vel_sub = rospy.Subscriber(namespace + '/est_vel', Float64, self.callback)  # Topic data from /est_vel
        self.est_vel = 0.0  # Holds most recent estimate published to /est_vel
        self.vel_history = [0.0, 0.0, 0.0, 0.0, 0.0]  # Set of previous velocity estimates. Initialize all to be 0.

    def callback(self, data):
        """
        callback function to handle incoming data from the /est_vel topic
        -----
        :param data: the received data from the topic (of type std_msgs/Float64)
        """
        self.est_vel = data.data
        self.update_velocity()  # Triggers the class to update the current velocity estimation based on the new data

    def get_velocity(self):
        """
        Returns the current velocity estimate
        """
        return self.vel

    def update_velocity(self):
        """
        Updates the v
        """
        self.vel_history[:-1] = self.vel_history[1:]
        self.vel_history[-1] = self.est_vel
        self.vel = self.est_avg_velocity()

    def est_avg_velocity(self):
        """
        Estimate the average velocity using the current data. More weight is placed on more recent estimates.
        """
        factors = [0.05, 0.075, 0.125, 0.25, 0.5]  # Multiplicative factors for each estimate of the
        # self.vel_history. Note that more weight is placed on more recent estimates from the /est_vel topic. These
        # factors can be tuned accordingly.
        # todo: Update factors to operate with no multiplicative restrictions. Currently, factors must all sum to 1
        #  to balance resulting velocity estimate.
        return sum([self.vel_history[i] * factors[i] for i in range(len(factors))]) # Return the new average.
