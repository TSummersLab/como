import os
import csv
from math import sqrt
from file_handling import *

package_path = find_package_path("test_launches")

fileName = os.path.join(package_path, 'data', 'logs', 'otsl_pos2023-08-16_15:05:12.csv')

with open(fileName, 'r') as f:
    reader = csv.reader(f)
    next(reader)
    points, timestamps, velocities = [], [], []
    for row in reader:
	x, y, heading, w_x, w_y, t, velocity, motor, servo, motor_gain, steering_gain = map(float, row)
	points.append([x, y])
	timestamps.append(t)
	velocities.append(velocity)

total_dist = 0.0

for i in range(1, len(points)):
    x1, y1 = points[i -1]
    x2, y2 = points[i]
    distance = sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    total_dist += distance

print("Total distance: ", total_dist)

time_elapsed = (timestamps[-1] - timestamps[9]) / (10 ** 9)
avg_vel = total_dist/time_elapsed

print("Average velocity: ", avg_vel)

total_dist2 = 0.0
for i in range(14, len(velocities)):
    avg_vel = (velocities[i - 1] + velocities[i])/2
    time_diff = (timestamps[i] - timestamps[i - 1]) / (10 ** 9)
    total_dist2 += avg_vel * time_diff

print("Total distance (by velocity): ", total_dist2)
