from Reader import DataReader as sr
import EKFSlam as ekf
import FastSLAM as fast
from matplotlib import pyplot as plot
import numpy as np
import math as math

# original motion model
# def get_result_matrix(input_matrix, sensor_data):
#     result_matrix = [[0] for j in range(3)]
#     result_matrix[0][0] = input_matrix[0] + sensor_data.data2 * math.cos(input_matrix[2] + sensor_data.data1)
#     result_matrix[1][0] = input_matrix[1] + sensor_data.data2 * math.sin(input_matrix[2] + sensor_data.data1)
#     result_matrix[2][0] = input_matrix[2] + sensor_data.data1 + sensor_data.data3
#     return np.array(result_matrix).transpose()

def cal_odom(read_data1, read_data):
    diff_x = read_data1.data1 - read_data.data1
    diff_y = read_data1.data2 - read_data.data2
    diff_angz = read_data1.data3 - read_data.data3
    r = math.sqrt(math.pow(diff_x, 2) + math.pow(diff_y, 2))
    #zeta = math.atan2(diff_y, diff_x)
    odom = np.array([r,diff_angz])
    return odom

def get_result_matrix(input_matrix, odom):
    result_matrix = [[0] for j in range(3)]
    result_matrix[0][0] = input_matrix[0] + odom[0] * math.cos(input_matrix[2] + odom[1])
    result_matrix[1][0] = input_matrix[1] + odom[0] * math.sin(input_matrix[2] + odom[1])
    result_matrix[2][0] = input_matrix[2] + odom[1]
    return np.array(result_matrix).transpose()

sensor_list = sr.sensor_reader()
data_list = sr.data_reader()
i = 0

f, ax = plot.subplots(nrows=1)
ax.set_title('Real Path')
ax.set_xlim(-1, 3)
ax.set_ylim(-1, 3)

robot_pose = np.zeros((3,1))
robot_pose[2][0] = 0

observations = list()

while i < len(sensor_list):
    if sensor_list[i].read_type == 'ODOMETRY':
        observations.append(sensor_list[i])
    i += 1

for k in range(len(observations)-1):
    read_data = observations[k]
    read_data1 = observations[k+1]
    odom = cal_odom(read_data1, read_data)
    robot_pose = get_result_matrix(robot_pose, odom)
    robot_pose = robot_pose.transpose()
    #ax.scatter(read_data.data1, read_data.data2)
    ax.scatter(robot_pose[0], robot_pose[1], color='g')

f.canvas.draw()
plot.pause(0.0000001)

plot.show()

