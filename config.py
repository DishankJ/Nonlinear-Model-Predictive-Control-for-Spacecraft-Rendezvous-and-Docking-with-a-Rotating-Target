import numpy as np

m = 1.0
I_z = 1.0
rad_target = 0.65
rad_chaser = 0.1
omega = 1
Ts = 0.01
T_horizon = 10
sim_time = 1000
rad_hold = 1.15
safety_dist = 0.03
gamma = 0.1
alpha = 0.95

u_max = 20*np.array([1, 1, 1])
u_min = -u_max

x0 = np.array([0, 2, -np.pi/2, 0, 0, 0])

# Q = np.diag([1600, 1600, 0, 10, 10, 400])
Q = np.diag([1600, 1600, 0, 10, 10, 400])
P = np.copy(Q)
R = np.array([0, 0, 0])

obs_on = True
obs = [(-0.5, 1, 0.2),
       (-1, -0.5, 0.2)]