from vehicle_data.topics_data import MotiveSub, EstVelSub, EstVelPub

class VehicleSub:
    '''
    Subscribes to all vehicle data from external systems and sensors
    '''

    def __init__(self, namespace, pose=None, orientation=None, velocity=None):
        self.motive_sub = MotiveSub(namespace, pose, orientation)
        self.est_vel_sub = EstVelSub(namespace, velocity)
        self.est_vel_pub = EstVelPub(namespace)

