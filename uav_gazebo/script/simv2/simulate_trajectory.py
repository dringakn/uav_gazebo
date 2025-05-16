#!/usr/bin/env python3
"""
track_trajectory.py
===================

Quadrotor trajectory-tracking demo.

The vehicle starts on a horizontal circle of radius 2 m at 5 m height and is
commanded to follow that circle twice (period 10 s ⇒ 20 s total).  Position
reference, velocity feed-forward, and yaw (pointing along the velocity vector)
are recomputed each step.  Results are plotted after the run.

Requires:
    params.py, state.py, math_utils.py, dynamics.py, controller.py, integrator.py
"""

import numpy as np
import matplotlib.pyplot as plt

from params     import QuadParams
from state      import QuadState
from dynamics   import QuadDynamics
from controller import CascadedPD, PDGains
from integrator import RK4
from math_utils import safe_normalise

# ---------------------------------------------------------------------
# Sim settings & objects
# ---------------------------------------------------------------------
dt, T = 0.005, 20.0                   # 4 kHz loop, 20 s sim
steps  = int(T/dt) + 1

phys   = QuadParams()
dyn    = QuadDynamics(phys)
gains  = PDGains(kp_pos=1.5, kd_pos=2.5,   # tighter than hover
                 kp_att=30. , kd_att=12.)
ctl    = CascadedPD(phys, gains)
ode    = RK4(dyn.rhs_vec)

# ---------------------------------------------------------------------
# Initial state (start on the circle, level attitude)
# ---------------------------------------------------------------------
x = QuadState(
    pos   = np.array([2.0, 0.0, 5.0]),
    vel   = np.zeros(3),
    quat  = np.array([0, 0, 0, 1]),
    omega = np.zeros(3),
).as_vec()

# ---------------------------------------------------------------------
# Reference trajectory (circle in XY, constant Z)
# ---------------------------------------------------------------------
R        = 2.0                       # m
T_circle = 10.0                       # s  ⇒ ω = 2π/T
ω_traj   = 2*np.pi / T_circle
z_ref    = 5.0

def reference(t: float):
    """Return (pos_ref, vel_ref, yaw_ref) at time t."""
    pos = np.array([ R*np.cos(ω_traj*t),
                     R*np.sin(ω_traj*t),
                     z_ref ])
    vel = np.array([-R*ω_traj*np.sin(ω_traj*t),
                     R*ω_traj*np.cos(ω_traj*t),
                     0.0 ])
    yaw = np.arctan2(vel[1], vel[0])          # face forward
    return pos, vel, yaw

# ---------------------------------------------------------------------
# Logging buffers
# ---------------------------------------------------------------------
time_log      = np.zeros(steps)
pos_log       = np.zeros((steps, 3))
pos_ref_log   = np.zeros((steps, 3))

# ---------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------
for k in range(steps):
    t   = k*dt
    s   = QuadState.from_vec(x)

    p_ref, v_ref, psi_ref = reference(t)
    u   = ctl(t, s, p_ref, v_ref, psi_ref)
    x   = ode.step(x, u, dt)
    q = x[6:10]
    x[6:10] = safe_normalise(q)

    # log
    time_log[k]    = t
    pos_log[k]     = x[0:3]
    pos_ref_log[k] = p_ref

# ---------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------
fig, ax = plt.subplots(3, 1, figsize=(8,6), sharex=True)
lbls = ['x [m]', 'y [m]', 'z [m]']
for i in range(3):
    ax[i].plot(time_log, pos_ref_log[:, i], 'k--', label='ref')
    ax[i].plot(time_log, pos_log[:, i],           label='actual')
    ax[i].set_ylabel(lbls[i])
    ax[i].legend()
ax[-1].set_xlabel('time [s]')
fig.suptitle('Quadrotor trajectory tracking: circular path (R = 2 m, z = 5 m)')
fig.tight_layout()
plt.show()

plt.figure(figsize=(5,5))
plt.plot(pos_ref_log[:,0], pos_ref_log[:,1], 'k--', label='ref')
plt.plot(pos_log[:,0],     pos_log[:,1],           label='actual')
plt.gca().set_aspect('equal', adjustable='box')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('XY path')
plt.legend()
plt.show()
