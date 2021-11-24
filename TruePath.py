from Reader import DataReader as sr
import EKFSlam as ekf
import FastSLAM as fast
from matplotlib import pyplot as plot
import numpy as np

sensor_list = sr.sensor_reader()
data_list = sr.data_reader()
i = 0

f, ax = plot.subplots(nrows=1)
ax.set_xlim(-2.5, 0)
ax.set_ylim(-1, 3)
while i < len(sensor_list):
    observations = list()
    i += 1
    while i < len(sensor_list) and sensor_list[i].read_type == 'ODOMETRY':
        observations.append(sensor_list[i])
        i += 1
    for k in range(len(observations)):
        read_data = observations[k]
        ax.scatter(read_data.data1, read_data.data2)

    
    f.canvas.draw()
    plot.pause(0.0000001)

plot.show()
