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

particles = np.random.rand(3, 100)
weights = np.zeros(100)
et_list = list()
q = [[0.1, 0], [0, 0.1]]

for z in range(100):
    et_list.append([[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]])

f, ax = plot.subplots(nrows=1)
k, bx = plot.subplots(nrows=1)
ax.set_xlim(-2.5, 0)
ax.set_ylim(-1, 3)
bx.set_xlim(-2.5, 0)
bx.set_ylim(-1, 3)
while i < len(sensor_list):
    observations = list()
    observations.append(sensor_list[i])
    i += 1
    while i < len(sensor_list) and sensor_list[i].read_type != 'ODOMETRY':
        observations.append(sensor_list[i])
        i += 1
    mut, et = ekf.calculate_odo(mut, et, observations, data_list)
    particles, et_list, weights = fast.eval_sensor_model(particles, et_list, weights, observations, data_list)
    #change the parameter to the turtlebot initial pose and right unit
    mut_list = []
    for items in mut:
        for item in items:
            mut_list.append(float(item))
    ax.scatter(particles[1]/5-2, -particles[0]/5, c=particles[2])
    bx.scatter(mut_list[1]/5-2, -mut_list[0]/5, color='b')
    f.canvas.draw()
    k.canvas.draw()
    plot.pause(0.0000001)

plot.show()
