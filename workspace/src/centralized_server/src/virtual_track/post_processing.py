import os
import matplotlib.pyplot as plt
import csv

def save_track(package_path, timestamp, motive_data, waypoints, timestamps, velocity, control_inputs):
    headers = ['x', 'y', 'heading', 'waypoint_x', 'waypoint_y', 'timestamp', 'velocity', 'throttle', 'servo_angle', 'motor_gain', 'servo_gain']
    data = []
    for i in range(len(control_inputs)):
        row = []
        row.append(motive_data[i][0])
        row.append(motive_data[i][1])
        row.append(motive_data[i][2])
        row.append(waypoints[i][0])
        row.append(waypoints[i][1])
        row.append(timestamps[i])
        row.append(velocity[i])
	row.append(control_inputs[i][0])
	row.append(control_inputs[i][1])
	row.append(control_inputs[i][2])
	row.append(control_inputs[i][3])
	data.append(row)

    fileName = "otsl_pos" + timestamp + ".csv"
    path = os.path.join(package_path, 'data', 'logs', fileName)
    writer = csv.writer(open(path, 'w'), delimiter=',', lineterminator='\n')
    writer.writerow(headers)
    writer.writerows(data)

def plot_track(x, y, x_track, y_track, timestamp, package_path):
    plt.clf()
    plt.plot(x, y, "k", alpha=0.2)
    plt.plot(x_track, y_track, "m")
    plt.axis('equal')
    fileName = "otsl_overlay" + timestamp + ".png"
    path = os.path.join(package_path, 'data', 'figures', fileName)
    plt.savefig(path)

def plot_velocity(velocity, timestamp, package_path):
    plt.clf()
    plt.plot(velocity[1000:])
    fileName = "otsl_velocity" + timestamp + ".png"
    path = os.path.join(package_path, 'data', 'figures', 'velocity', fileName)
    plt.savefig(path)

def plot_complete_velocity(velocity, timestamp, package_path):
    plt.clf()
    plt.plot(velocity)
    fileName = "otsl_velocity" + timestamp + "_complete.png"
    path = os.path.join(package_path, 'data', 'figures', 'velocity', fileName)
    plt.savefig(path)

def plot_track_deviation(track_deviation, timestamp, package_path):
    plt.clf()
    plt.plot(track_deviation)
    fileName = "otsl_error" + timestamp + "_track_deviation.png"
    path = os.path.join(package_path, 'data', 'figures', 'track_deviation', fileName)
    plt.savefig(path)

def plot_data(data, timestamp, package_path, relative_path="", trim_count=0, data_description="", data_description2=""):
    plt.clf()
    data = data[trim_count:]
    plt.plot(data)
    fileName = "otsl_" + data_description + timestamp + data_description2 + ".png"
    path = os.path.join(package_path, 'data', 'figures', relative_path, fileName)
    plt.savefig(path)
