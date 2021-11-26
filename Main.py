from Reader import DataReader as sr
import EKFSlam as ekf
import FastSLAM as fast
from matplotlib import pyplot as plot
import numpy as np

sensor_list = sr.sensor_reader()
data_list = sr.data_reader()
i = 0
length = 2 * len(data_list) + 3
mut = ekf.initialize(len(data_list))
et = [[0 for i in range(length)] for j in range(length)]

particles = np.zeros((3,100))
particles[0,:] = 0.1*(np.random.rand(1, 100)-0.5)
particles[1,:] = 0.1*(np.random.rand(1, 100)-0.5)
particles[2,:] = 0.1*(np.random.rand(1, 100))

robot_pose = particles[:,0]
weights = np.zeros(100)
et_list = list()

q = [[0.1, 0], [0, 0.1]]

for z in range(100):
    et_list.append([[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]])

f, ax = plot.subplots(nrows=1)
k, bx = plot.subplots(nrows=1)
g, cx = plot.subplots(nrows=1)
ax.set_title('FastSlam1.0 (With Particles)')
cx.set_title('FastSlam1.0')
bx.set_title('EKFSlam')
ax.set_xlim(-1, 3)
ax.set_ylim(-1, 3)
cx.set_xlim(-1, 3)
cx.set_ylim(-1, 3)
bx.set_xlim(-1, 3)
bx.set_ylim(-1, 3)
added_landmarks = list()
read_data = sensor_list[0]
while i < len(sensor_list):
    observations = list()
    observations.append(sensor_list[i])
    i += 1
    while i < len(sensor_list) and sensor_list[i].read_type != 'ODOMETRY':
        observations.append(sensor_list[i])
        i += 1
    
    mut, et = ekf.calculate_odo(mut, et, read_data, observations, data_list)
    particles, et_list, weights, added_landmarks, robot_pose = fast.eval_sensor_model(particles, et_list, weights, read_data, observations, data_list, added_landmarks, robot_pose)
    mut_list = []
    for items in mut:
        for item in items:
            mut_list.append(float(item))

    read_data = observations[0]
    ax.scatter(particles[0], particles[1], c =particles[1])
    ax.scatter(robot_pose[0], robot_pose[1], color='r')
    bx.scatter(mut_list[0], mut_list[1], color='b')
    cx.scatter(robot_pose[0], robot_pose[1], color='r')
    f.canvas.draw()
    k.canvas.draw()
    g.canvas.draw()
    plot.pause(0.0000001)

plot.show()
