#!/usr/bin/env python3
"""
Author: Dr. -Ing. Ahmad Kamal Nasir <dringakn@gmail.com>

Quadrotor Simulator – Hover Test

This script performs a six‐degree‐of‐freedom simulation of a quadrotor UAV using
a cascaded PD control architecture.  It demonstrates a simple hover at 5 m
altitude in the world frame for a duration of 10 s.

Key Features:
  - Rigid‐body dynamics with quaternion‐based attitude representation
  - Cascaded PD controller:
      • Outer loop: position control (X, Y, Z)
      • Inner loop: attitude control (roll, pitch, yaw)
  - Runge–Kutta 4th‐order integrator for time stepping
  - Headless logging of position & attitude, followed by Matplotlib plotting

Simulation Setup:
  • Time step (dt): 0.005 s  
  • Total duration (T): 10 s  
  • Initial state: at origin, level attitude, zero velocity & angular rate  
  • Reference: hover point at (0, 0, 5 m), zero yaw rate  

Usage:
    python simulate.py

"""
import numpy as np
import matplotlib.pyplot as plt

from params     import QuadParams
from state      import QuadState
from dynamics   import QuadDynamics
from controller import CascadedPD, PDGains
from integrator import RK4

# --- simulation setup -------------------------------------------------
dt, T = 0.005, 10.0  # time step [s], total time [s]
steps = int(T / dt)

p   = QuadParams()
dyn = QuadDynamics(p)
ctl = CascadedPD(p, PDGains())
ode = RK4(dyn.rhs_vec)

# initial state: on ground, level
x0 = QuadState(
    pos   = np.zeros(3),
    vel   = np.zeros(3),
    quat  = np.array([0, 0, 0, 1]),   # no rotation
    omega = np.zeros(3),
).as_vec()

# references (constant)
pos_ref = np.array([0, 0, 5])
vel_ref = np.zeros(3)
yaw_ref = 0.0

# --- main loop --------------------------------------------------------
hist = {"t": [], "pos": [], "quat": []}
x = x0.copy()
for k in range(steps):
    t  = k * dt
    s  = QuadState.from_vec(x)

    u  = ctl(t, s, pos_ref, vel_ref, yaw_ref)
    x  = ode.step(x, u, dt)

    # log
    hist["t"].append(t)
    hist["pos"].append(x[0:3].copy())
    hist["quat"].append(x[6:10].copy())

# --- plotting ---------------------------------------------------------
t_arr   = np.array(hist["t"])
pos_arr = np.vstack(hist["pos"])

fig, ax = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
labels  = ['x [m]', 'y [m]', 'z [m]']
for i in range(3):
    ax[i].plot(t_arr, pos_arr[:, i])
    ax[i].set_ylabel(labels[i])
ax[-1].set_xlabel('time [s]')
fig.suptitle("Quadrotor Position vs. Time")
fig.tight_layout()
plt.show()
