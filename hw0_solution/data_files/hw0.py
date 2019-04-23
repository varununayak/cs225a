
# coding: utf-8

# In[2]:


import numpy as np
from sympy import *
import matplotlib.pyplot as plt
get_ipython().run_line_magic('matplotlib', 'inline')
get_ipython().run_line_magic('config', "InlineBackend.figure_format = 'svg'")


# In[3]:


L, q1, d2, q3 = symbols("L q1 d2 q3")
J = Matrix([
    [L*sin(q1)*sin(q3),  0, -L*cos(q1)*cos(q3)],
    [-L*cos(q1)*sin(q3), 0, -L*sin(q1)*cos(q3)],
    [0,                  1, -L*sin(q3)]
   ])
M = J.transpose() @ J
M.simplify()
M


# In[4]:


# 2c

P1 = (0, 1, 1)
P2 = (-1, 0, 1)


# In[5]:


# 2d

J1 = np.array([
    [-1, 0, 0],
    [0, 0, 0],
    [0, 1, 1]
])

J2 = np.array([
    [0, 0, 0],
    [-1, 0, 0],
    [0, 1, 1]
])


# In[37]:


# 2e

q3_dense = np.linspace(-90,90)
q3 = (-90, -60, -30, 0, 30, 60, 90)
m11 = (1, 0.75, 0.25, 3e-7, 0.25, 0.75, 1)
m22 = (1.01, 1.01, 1.01, 1.01, 1.01, 1.01, 1.01)
m33 = (1, 1, 1, 1, 1, 1, 1)
G1 = np.zeros(len(q3))
G2 = 9.9081 * np.ones(len(q3))
G3 = (9.81, 8.49571, 4.905, 0, -4.905, -8.49571, -9.81)

d2 = (0, 0.5, 1, 1.5, 2)
m11 = 3e-7 * np.ones(len(d2))
m22 = 1.01 * np.ones(len(d2))
m33 = np.ones(len(d2))
G1 = np.zeros(len(d2))
G2 = 9.9081 * np.ones(len(d2))
G3 = np.zeros(len(d2))

fig = plt.figure(figsize=(8,10))

ax1 = fig.add_subplot(211)
# ax1.plot(q3, m11, label=r'$m_{11}$')
# ax1.plot(q3, m22, label=r'$m_{22}$')
# ax1.plot(q3, m33, label=r'$m_{33}$')
ax1.plot(q3_dense, np.sin(np.deg2rad(q3_dense))**2, label=r'$m_{11}$')
ax1.plot(q3_dense, 1.01 * np.ones(len(q3_dense)), label=r'$m_{22}$')
ax1.plot(q3_dense, np.ones(len(q3_dense)), label=r'$m_{33}$')

ax1.set_xticks(q3)
ax1.set_xlabel(r'$\theta_3$')
ax1.set_ylabel('Mass matrix element')
ax1.set_title(r'Effect of varying $\theta_3$')

ax1.legend(loc=(1.1,0.5),frameon=1)

ax2 = fig.add_subplot(212)
ax2.plot(d2, m11, label=r'$m_{11}$')
ax2.plot(d2, m22, label=r'$m_{22}$')
ax2.plot(d2, m33, label=r'$m_{33}$')

ax2.set_xticks(d2)
ax2.set_xlabel(r'$d_2$')
ax2.set_ylabel('Mass matrix element')
ax2.set_title(r'Effect of varying $d_2$')

lgd = ax2.legend(loc=(1.1,0.5),frameon=1)

plt.savefig('Plots/mass.png', dpi=300, bbox_extra_artists=(lgd,), bbox_inches='tight')

plt.show()


# In[38]:


# 2f

q3_dense = np.linspace(-90,90)
q3 = (-90, -60, -30, 0, 30, 60, 90)
m11 = (1, 0.75, 0.25, 3e-7, 0.25, 0.75, 1)
m22 = (1.01, 1.01, 1.01, 1.01, 1.01, 1.01, 1.01)
m33 = (1, 1, 1, 1, 1, 1, 1)
G1 = np.zeros(len(q3))
G2 = 9.9081 * np.ones(len(q3))
G3 = (9.81, 8.49571, 4.905, 0, -4.905, -8.49571, -9.81)

d2 = (0, 0.5, 1, 1.5, 2)
m11 = 3e-7 * np.ones(len(d2))
m22 = 1.01 * np.ones(len(d2))
m33 = np.ones(len(d2))
G1 = np.zeros(len(d2))
G2 = 9.9081 * np.ones(len(d2))
G3 = np.zeros(len(d2))

fig = plt.figure(figsize=(8,10))

ax1 = fig.add_subplot(211)
# ax1.plot(q3, G1, label=r'$G_{1}$')
# ax1.plot(q3, G2, label=r'$G_{2}$')
# ax1.plot(q3, G3, label=r'$G_{3}$')
ax1.plot(q3_dense, 9.81 * 1.01 * np.ones(len(q3_dense)), label=r'$G_{1}$')
ax1.plot(q3_dense, np.zeros(len(q3_dense)), label=r'$G_{2}$')
ax1.plot(q3_dense, -9.81 * np.sin(np.deg2rad(q3_dense)), label=r'$G_{3}$')

ax1.set_xticks(q3)
ax1.set_xlabel(r'$\theta_3$')
ax1.set_ylabel('Gravitational torque')
ax1.set_title(r'Effect of varying $\theta_3$')

ax1.legend(loc=(1.1,0.5),frameon=1)

ax2 = fig.add_subplot(212)
ax2.plot(d2, G1, label=r'$G_{1}$')
ax2.plot(d2, G2, label=r'$G_{2}$')
ax2.plot(d2, G3, label=r'$G_{3}$')

ax2.set_xticks(d2)
ax2.set_xlabel(r'$d_2$')
ax2.set_ylabel('Gravitational torque')
ax2.set_title(r'Effect of varying $d_2$')

lgd = ax2.legend(loc=(1.1,0.5),frameon=1)

plt.savefig('Plots/gravity.png', dpi=300, bbox_extra_artists=(lgd,), bbox_inches='tight')

plt.show()
