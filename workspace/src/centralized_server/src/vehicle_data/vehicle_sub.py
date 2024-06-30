from vehicle_data.topics_data import MotiveSub, EstVelSub, EstVelPub

class VehicleSub:
    """
    Subscribes to all vehicle data from external systems and sensors
    """

    def __init__(self, namespace, pose=None, orientation=None, velocity=None):
        """
        Initialize the VehicleSub class, which maintains all required data for a given vehicle. Utilizes different
        modules, sensors, and related subsystems to update and utilize this data. This version depends on an
        Optitrack-based system and a velocity estimation module based on collected Optitrack data.
        ----
        :param namespace: namespace of the vehicle
        :param pose: initial pose of the vehicle (Point), None if not included
        :param orientation: initial orientation of the vehicle (Orientation), None if not included
        :param velocity: initial velocity of the vehicle, None if not included
        """

        # Initialize the submodules responsible for collecting and computing vehicle data
        self.motive_sub = MotiveSub(namespace, pose, orientation)
        self.est_vel_sub = EstVelSub(namespace, velocity)
        self.est_vel_pub = EstVelPub(namespace)

