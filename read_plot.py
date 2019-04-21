

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
m = []


dt = 0.001	#loop frequency

with open('/media/varun/Work/Academics/_Spring 2019/CS 225A/cs225a_hw2/data4i.csv','r') as csvfile:
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

       	k.append(float(row[10]))
       	l.append(float(row[11]))
       	m.append(float(row[12]))



t = np.linspace(0,len(a)*dt,len(a),dtype=float)

plt.figure(0)
plt.plot(t,a, label='x')
plt.plot(t,b, label='y')
plt.plot(t,c, label='z')

plt.plot(t,k, label='x_des')
plt.plot(t,l, label='y_des')
plt.plot(t,m, label='z_des')

plt.xlabel('Time [s]')
plt.ylabel('Position Operation Space [m]')
plt.title('Task Space Trajectory')
plt.legend()
plt.grid()

plt.savefig('/media/varun/Work/Academics/_Spring 2019/CS 225A/cs225a_hw2/plot4i_task.png')

plt.figure(1)
plt.plot(t,d, label='Joint 1')
plt.plot(t,e, label='Joint 2')
plt.plot(t,f, label='Joint 3')
plt.plot(t,g, label='Joint 4')
plt.plot(t,h, label='Joint 5')
plt.plot(t,i, label='Joint 6')
plt.plot(t,j, label='Joint 7')


plt.xlabel('Time [s]')
plt.ylabel('Angle [rad]')
plt.title('Joint Space Trajectory')
plt.legend()
plt.grid()

plt.savefig('/media/varun/Work/Academics/_Spring 2019/CS 225A/cs225a_hw2/plot4i_joint.png')



plt.show()
