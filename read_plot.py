

import matplotlib.pyplot as plt
import csv
import numpy as np

x = []
y = []
z = []

dt = 0.001	#loop frequency

with open('/media/varun/Work/Academics/_Spring 2019/CS 225A/cs225a_hw1/joint_trajectory_6.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        x.append(float(row[0]))
        y.append(float(row[1]))
        z.append(float(row[2]))

t = np.linspace(0,len(x)*dt,len(x),dtype=float)
plt.plot(t,x, label='Joint 1')
plt.plot(t,y, label='Joint 3')
plt.plot(t,z, label='Joint 4')
plt.xlabel('Time [s]')
plt.ylabel('Angle [rad]')
plt.title('Joint Trajectory for Q6')
plt.legend()
plt.grid()

plt.savefig('/media/varun/Work/Academics/_Spring 2019/CS 225A/cs225a_hw1/q6_plot.png')
plt.show()
