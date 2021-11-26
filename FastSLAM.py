import numpy as np
from numpy.linalg import inv
import math
from random import random

# calculate odometry
# def get_result_matrix(input_matrix, sensor_data):
#     result_matrix = [[0] for j in range(3)]
#     result_matrix[0][0] = input_matrix[0] + sensor_data.data2 * math.cos(input_matrix[2] + sensor_data.data1)
#     result_matrix[1][0] = input_matrix[1] + sensor_data.data2 * math.sin(input_matrix[2] + sensor_data.data1)
#     result_matrix[2][0] = input_matrix[2] + sensor_data.data1 + sensor_data.data3
#     return np.array(result_matrix).transpose()

def get_result_matrix(input_matrix, odom):
    result_matrix = [[0] for j in range(3)]
    result_matrix[0][0] = input_matrix[0] + odom[0] * math.cos(input_matrix[2] + odom[1])
    result_matrix[1][0] = input_matrix[1] + odom[0] * math.sin(input_matrix[2] + odom[1])
    result_matrix[2][0] = input_matrix[2] + odom[1]
    return np.array(result_matrix).transpose()

def cal_odom(read_data1, read_data):
    diff_x = read_data1.data1 - read_data.data1
    diff_y = read_data1.data2 - read_data.data2
    diff_angz = read_data1.data3 - read_data.data3
    r = math.sqrt(math.pow(diff_x, 2) + math.pow(diff_y, 2))
    #zeta = math.atan2(diff_y, diff_x)
    odom = np.array([r,diff_angz])
    return odom

def multiply_multi(*args):
    first = np.array(args[0])
    second = np.array(args[1])
    third = np.array(args[2])
    total = first.dot(second)
    return np.array(total).dot(third)

# compute z in (r, zeta) from delta x delta y
def get_sensor_result(result, readed):
    result_matrix = [[0] for j in range(2)]
    mx = readed.x
    my = readed.y
    result_matrix[0][0] = math.sqrt(math.pow(mx - result[0], 2) + math.pow(my - result[1], 2))
    result_matrix[1][0] = math.atan((my - result[1]) / (mx - result[0])) - result[2]
    return result_matrix

def h_jacob(result, readed):
    mx = readed.x
    my = readed.y
    xt = result[0]
    yt = result[1]
    result_bot = 1 + math.pow((my - yt) / (mx - xt), 2)
    result_matrix = [[xt - mx, yt - my, 0.0],
                     [(1 / math.pow(mx - xt, 2)) / result_bot, -1 / result_bot, -1.0]]
    return result_matrix

def eval_sensor_model(particles, et_list, weights, read_data, observations, data_list, added_landmarks, robot_pose):
    # apply motion model

    read_data1 = observations[0]
    odom = cal_odom(read_data1, read_data)

    for index in range(100):
        particles[:, index] = get_result_matrix(particles[:, index], odom)
        #particles[:, index] = get_result_matrix(particles[:, index], observations[0])

    robot_pose = get_result_matrix(robot_pose, odom)
    #obot_pose = get_result_matrix(robot_pose, observations[0])

    I = np.identity(3)
    noise = 0.001 #measurement noise
    qt = np.array([[noise, 0], [0, noise]])
    qt3 = np.array([[noise, 0, 0], [0, noise, 0], [0, 0, noise]])
    p0 = 1 # initial weight
    for index in range(100):
        weights[index] = 0
        for k in range(len(observations) - 1):
            tempw = 0
            read_data = observations[k + 1]
            particle = particles[:, index]
            if read_data.data1 not in added_landmarks:
                H_MATRIX = h_jacob(particle, data_list[int(read_data.data1 - 1)])
                H_MATRIX.append([0.0, 0.0, 0.0])
                et = multiply_multi(np.linalg.pinv(H_MATRIX), qt3, np.array(np.linalg.pinv(H_MATRIX)).transpose())
                et_list[index] = et
                weights[index] = p0
            else:
                H_MATRIX = h_jacob(particle, data_list[int(read_data.data1 - 1)])
                z = get_sensor_result(robot_pose[0],  data_list[int(read_data.data1 - 1)]) # compute z
                zhat = get_sensor_result(particle,  data_list[int(read_data.data1 - 1)]) # compute z hat
                diffz = np.subtract(z ,zhat)
                Q = np.add(multiply_multi(H_MATRIX, et_list[index], np.array(H_MATRIX).transpose()), qt)
                K = multiply_multi(et_list[index], np.array(H_MATRIX).transpose(), inv(Q))
                KH = np.matmul(np.array(K), H_MATRIX)
                et_list[index] = np.matmul(np.subtract(I, KH), et_list[index])
                qu = np.array(2 * np.pi * Q)
                temp = np.matmul(np.matmul(diffz.transpose(), inv(Q)), diffz)
                tempw = (np.linalg.det(qu)**(-0.5))*np.exp(-0.5*temp)
                weights[index] += tempw

    max_index = np.argmax(weights)
    robot_pose = particles[:, max_index]
    print(weights[max_index])

    for k in range(len(observations) - 1):
        read_data = observations[k + 1]
        if read_data.data1 not in added_landmarks:
            added_landmarks.append(int(read_data.data1))    
    return resample_particles(particles, weights,), et_list, weights, added_landmarks, robot_pose 

def resample_particles(particles, weights):
    y = sum(weights)
    n, m = particles.shape
    step = sum(weights) / m
    u = np.random.uniform(0, step)
    c = weights[0]
    z = 0
    i = 0
    new_particles = np.zeros((3,100))
    for n in range(m-1):
        while u > c:
            i = i + 1
            if i == 100:
                return np.array(new_particles)
            c = c + weights[i]
        new_particles[0,n] = particles[0,i] + 0.2*(random()-0.5)
        new_particles[1,n] = particles[1,i] + 0.2*(random()-0.5)
        new_particles[2,n] = particles[2,i] + 0.1*(random()-0.5)
        weights[z] = 1.0 / m
        u = u + step
        z+=1
    return np.array(new_particles)
