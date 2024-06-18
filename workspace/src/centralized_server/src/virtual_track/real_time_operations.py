from Queue import Queue
from rospy import Publisher
from math import sqrt
from datetime import datetime
import numpy as np
#from pykalman import KalmanFilter
import random

def calculate_vel(p1, p2, rate, prev_est):
    del_x = p2[0] - p1[0]
    del_y = p2[1] - p1[1]
    dis = sqrt(del_x ** 2+ del_y ** 2)
    vel_est = dis * rate
    if abs(vel_est - prev_est) > 0.5:
	vel_est = prev_est
    return vel_est # multiply by frequency of program

def trim_outliers(vel_est, trim_count):
    #return [i for i in vel_est if i > min(vel_est) and i < max(vel_est)]
    for i in range(trim_count):
	vel_est.remove(max(vel_est))
	vel_est.remove(min(vel_est))
    return vel_est

def shift_queue(q, est_vel, shift_num, trim_count, rand_remove, prev_est):
    points = list(q.queue)
    vel_est = []
    for i in range(q.qsize() - 1):
    #for i in range(q.qsize() - 3):
	del_time = (points[i + 1][1] - points[i][1])/(10.0 ** 9)
	if del_time == 0:
		del_time = 10000000
	vel_est.append(calculate_vel(points[i][0], points[i + 1][0], 1/del_time, prev_est))
    vel_est = trim_outliers(vel_est, trim_count)
    for i in range(rand_remove):
	vel_est.remove(random.choice(vel_est))
    avg_vel_est = sum(vel_est)/len(vel_est)
    #avg_vel_est = fir_velocity_filter(vel_est)
    est_vel.publish(avg_vel_est)
    for i in range(shift_num):
        robot_pos = q.get()

def generate_cur_timestamp():
    timestamp = datetime.now()
    formatted_timestamp = timestamp.strftime("%M:%S.%f")
    return formatted_timestamp

def time_difference(t1, t2):
    date_format = "%M:%S.%f"
    prev = datetime.strptime(t1, date_format)
    now = datetime.strptime(t2, date_format)
    return (now - prev).total_seconds() * 1000

def find_operation_time(t1, t2):
    t1 = t2
    t2 = generate_cur_timestamp()
    return t1, t2, time_difference(t1, t2)

