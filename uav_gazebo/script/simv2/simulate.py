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
steps = int(T / dt) + 1  # number of steps

p   = QuadParams() # vehicle parameters
gains  = PDGains(kp_pos=4, kd_pos=2.5,   
                 kp_att=50 , kd_att=20, # tighter than pos
                 kp_yaw=4, kd_yaw=2.5)
ctl = CascadedPD(p, gains)
dyn = QuadDynamics(p)
ode = RK4(dyn.rhs_vec)

# initial state: on ground, level
x0 = QuadState(
    pos   = np.zeros(3),
    vel   = np.zeros(3),
    quat  = np.array([0, 0, 0, 1]),   # no rotation
    omega = np.zeros(3),
).as_vec()

# references (constant)
pos_ref = np.array([0, 0, 10])
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

# --- performance metrics for z(t) ----------------------------------------
t_arr   = np.array(hist["t"])
pos_arr = np.vstack(hist["pos"])

z      = pos_arr[:,2]
final  = pos_ref[2]
tol    = 0.02*final  # 2% band

# Rise time: time from 10% to 90% of final value
t10 = t_arr[np.where(z >= 0.1*final)[0][0]]
t90 = t_arr[np.where(z >= 0.9*final)[0][0]]
rise_time = t90 - t10

# Peak time & % overshoot
peak_idx   = np.argmax(z)
peak_time  = t_arr[peak_idx]
overshoot  = (z[peak_idx] - final)/final * 100.0

# Settling time: first time after which it stays within ±2%
outside    = np.where(np.abs(z - final) > tol)[0]
if len(outside) > 0 and outside[-1] < len(t_arr)-1:
    settling_time = t_arr[outside[-1]+1]
else:
    settling_time = t_arr[-1]  # never leaves band after end

# Damping ratio ζ and natural frequency ωn (from standard second-order formulas)
# %OS = exp(-ζπ/√(1-ζ²)) ⇒ solve for ζ 
if overshoot > 0:
    zeta = -np.log(overshoot/100.0) / np.sqrt(np.pi**2 + np.log(overshoot/100.0)**2)
    wn   = np.pi / (peak_time * np.sqrt(1 - zeta**2))
else:
    zeta = wn = np.nan

# --- plotting ---------------------------------------------------
fig, ax = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
labels  = ['x [m]', 'y [m]', 'z [m]']
for i in range(3):
    ax[i].plot(t_arr, pos_arr[:, i])
    ax[i].set_ylabel(labels[i])
ax[-1].set_xlabel('time [s]')
fig.suptitle("Quadrotor Position vs. Time")
fig.tight_layout()
plt.connect('key_press_event', lambda event: plt.close() if event.key == 'escape' else None)
plt.show()

fig, ax = plt.subplots(figsize=(8,4))
ax.plot(t_arr, z,     label='z(t)')
ax.axhline(final,     linestyle='--', color='gray', label='5 m target')
# annotate metrics
ax.axvline(rise_time, linestyle=':', color='C1')
ax.text(rise_time, 0.1*final,    f"Tr = {rise_time:.2f}s",    rotation=90, va='bottom')
ax.axvline(peak_time, linestyle=':', color='C2')
ax.text(peak_time, 0.9*z[peak_idx], f"Tp = {peak_time:.2f}s\n%OS = {overshoot:.1f}%", 
        rotation=90, va='bottom')
ax.axvline(settling_time, linestyle=':', color='C3')
ax.text(settling_time, 0.5*final, f"Ts = {settling_time:.2f}s", rotation=90, va='bottom')

# footer with ζ and ωn
props = dict(boxstyle='round', facecolor='white', alpha=0.8)
ax.text(0.02, 0.05,
        f"ζ = {zeta:.2f}\nωₙ = {wn:.2f} rad/s",
        transform=ax.transAxes, bbox=props)

ax.set_xlabel('time [s]')
ax.set_ylabel('altitude z [m]')
ax.set_title("Hover Step Response & Performance Metrics")
ax.legend(loc='upper right')
fig.tight_layout()
plt.connect('key_press_event', lambda event: plt.close() if event.key == 'escape' else None)
plt.show()
