

import matplotlib.pyplot as plt
import csv
import numpy as np


a = []
b = []
c = []

d = []
e = []
f = []

g = []
h = []

i = []
j = []

k = []
l = []



dt = 0.001	#loop frequency (time per loop)

with open('/media/varun/Work/Academics/_Spring 2019/CS 225A/cs225a_hw3/data4b.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        a.append(float(row[0]))
       	b.append(float(row[1]))
       	c.append(float(row[2]))
       	
       	d.append(float(row[3]))
       	e.append(float(row[4]))
       	f.append(float(row[5]))

        g.append(float(row[6]))
        h.append(float(row[7]))
        i.append(float(row[8]))

        j.append(float(row[9]))
        k.append(-1*float(row[9]))



     



t = np.linspace(0,len(a)*dt,len(a),dtype=float)

plt.figure(0)

plt.subplot(2,1,1)
plt.plot(t,a, label='x')
plt.plot(t,b, label='y')
plt.plot(t,c, label='z')

plt.plot(t,d,'--', label='x_des')
plt.plot(t,e,'--' ,label='y_des')
plt.plot(t,f, '--', label='z_des')

plt.xlabel('Time [s]')
plt.ylabel('Position Operation Space [m]')
plt.title('Task Space Trajectory Tracking')
plt.legend()
plt.grid()


plt.subplot(2,1,2)

plt.plot(t,g, label='$\dot x_x$')
plt.plot(t,h,label='$\dot x_y$')
plt.plot(t,i, label='$\dot x_z$')

plt.plot(t,j,'r', label='$ V_{max}$')
plt.plot(t,k,'r', label='$ -V_{max}$')




plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.legend()
plt.grid()


plt.savefig('/media/varun/Work/Academics/_Spring 2019/CS 225A/cs225a_hw3/plot4b.png')






plt.show()
