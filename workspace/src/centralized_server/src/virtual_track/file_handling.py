import rospkg
from datetime import datetime

def find_package_path(package_name):
    rospack = rospkg.RosPack()
    package_path = rospack.get_path(package_name)
    return package_path

def generate_timestamp():
    timestamp = datetime.now()
    formatted_timestamp = timestamp.strftime("%Y-%m-%d_%H:%M:%S")
    return formatted_timestamp
