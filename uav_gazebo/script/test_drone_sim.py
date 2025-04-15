#!/usr/bin/env python3
"""
Author: [Dr. -Ing. Ahmad Kamal Nasir <dringakn@gmail.com>]

Quadrotor Simulator in Python

This simulator implements a basic closed-loop simulation for a quadrotor UAV.
It uses cascaded PD controllers for position (outer loop) and attitude (inner loop)
control. The simulation integrates both translational and rotational dynamics
using simple Euler integration.

--------------------------------------------------------------
System Parameters:
--------------------------------------------------------------
- Mass (m): 1.0 kg
- Inertia (I) in the body frame (I_b):
    I_b = [ [0.003,   0,     0  ],
            [  0,   0.003,   0  ],
            [  0,     0,   0.006] ]  (kg·m²)
- Gravity (g): [0, 0, -9.81] m/s² in the world frame

--------------------------------------------------------------
Controller Gains:
--------------------------------------------------------------
- Outer-loop (Position controller):
    kp_pos = 0.25, kd_pos = 1.0
- Inner-loop (Attitude controller for roll and pitch):
    kp_att = 25.0, kd_att = 10.0
- Yaw controller:
    kp_yaw = 2.25, kd_yaw = 3.0
- Angular velocity damping (if used):
    k_omega = 3.5

--------------------------------------------------------------
Reference Frames:
--------------------------------------------------------------
- World Frame:
    Inertial frame where the drone's position (pos) and linear velocity (vel)
    are defined. Gravity is expressed in this frame.
- Body Frame:
    Frame attached to the drone. The inertia matrix is defined in this frame,
    and the thrust is assumed to act along the positive z-axis of the body.
- Euler Angles:
    Represented as (phi, theta, psi) for roll, pitch, and yaw (in radians).
    They describe the orientation of the body frame with respect to the world frame.
    Here we use a ZYX (yaw-pitch-roll) convention for the rotation matrix.

--------------------------------------------------------------
State Variables:
--------------------------------------------------------------
- pos:       Position vector [x, y, z] in the world frame (meters).
- vel:       Linear velocity vector [v_x, v_y, v_z] in the world frame (m/s).
- euler:     Euler angles [phi, theta, psi] (radians) representing roll, pitch, and yaw.
- omega:     Angular velocity vector [p, q, r] in the body frame (rad/s).

--------------------------------------------------------------
Control Strategy:
--------------------------------------------------------------
1. Position Control:
   - Computes position error: pos_error = pos_des - pos.
   - Computes velocity error: vel_error = vel_des - vel.
   - Desired acceleration is computed via a PD controller:
         a_cmd = kp_pos * pos_error + kd_pos * vel_error.
   - The required force in the world frame is:
         F_des = m * (a_cmd - g),
     and the thrust command f_cmd is the magnitude ||F_des||.
     
2. Attitude Control:
   - From F_des, desired roll and pitch angles are computed:
         phi_des   = atan2(-F_des_y, F_des_z)
         theta_des = asin(F_des_x / f_cmd)
   - The desired Euler angles are then set as:
         euler_des = [phi_des, theta_des, psi_des],
     where psi_des is the desired yaw.
   - A PD controller is applied to track these angles (and their rates).

3. Dynamics Integration:
   - Translational dynamics:
         acc = (R * [0, 0, f_cmd]) / m + g,
     where R is the rotation matrix from body to world frame computed from Euler angles.
   - Rotational dynamics (Euler's equations):
         omega_dot = I⁻¹ * (torque - omega × (I * omega)),
     where torque is computed from the attitude PD controller.
     
4. State Integration:
   - The state (position, velocity, Euler angles, angular velocity) is updated using Euler integration.

--------------------------------------------------------------
Usage:
--------------------------------------------------------------
- Run the script with Python3.
- The simulation runs for a fixed time period and plots the evolution
  of the position and Euler angles over time.
  
"""

import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
dt = 0.005           # time step [s]
T  = 10              # total simulation time [s]
steps = int(T / dt)

# Drone physical parameters
mass = 1.0           # mass [kg]
I = np.diag([0.003, 0.003, 0.006])  # Inertia matrix in the body frame [kg*m^2]
I_inv = np.linalg.inv(I)
g = np.array([0, 0, -9.81])         # Gravity in the world frame [m/s^2]

# Controller gains
# Outer-loop (position control)
kp_pos = 0.25
kd_pos = 1.0

# Inner-loop (attitude control for roll and pitch)
kp_att = 25.0
kd_att = 10.0

# Yaw controller gains
kp_yaw = 2.25
kd_yaw = 3.0

# Angular velocity controller (optional damping)
k_omega = 3.5

# Desired state
pos_des = np.array([0, 0, 5])      # desired position in world frame (hover at 5 m altitude)
vel_des = np.array([0, 0, 0])      # desired velocity in world frame
psi_des = 0.0                      # desired yaw angle (radians)

# State initialization
pos = np.array([0, 0, 0], dtype=float)   # Position in world frame: [x, y, z]
vel = np.array([0, 0, 0], dtype=float)   # Linear velocity in world frame: [v_x, v_y, v_z]
euler = np.array([0, 0, 0], dtype=float)   # Euler angles: [roll (phi), pitch (theta), yaw (psi)]
omega = np.array([0, 0, 0], dtype=float)   # Angular velocity in body frame: [p, q, r]

# Data storage for plotting
pos_history = []
euler_history = []
time_history = []

def rotation_matrix(euler):
    """Compute rotation matrix from Euler angles (phi, theta, psi) using ZYX (yaw-pitch-roll) convention.
    
    Returns:
        R: 3x3 rotation matrix that transforms vectors from the body frame to the world frame.
    """
    phi, theta, psi = euler
    # Rotation about Z-axis (yaw)
    Rz = np.array([[np.cos(psi), -np.sin(psi), 0],
                   [np.sin(psi),  np.cos(psi), 0],
                   [0,            0,           1]])
    # Rotation about Y-axis (pitch)
    Ry = np.array([[np.cos(theta), 0, np.sin(theta)],
                   [0,             1, 0],
                   [-np.sin(theta),0, np.cos(theta)]])
    # Rotation about X-axis (roll)
    Rx = np.array([[1, 0,           0],
                   [0, np.cos(phi), -np.sin(phi)],
                   [0, np.sin(phi),  np.cos(phi)]])
    # Combined rotation matrix: R = Rz * Ry * Rx
    return Rz @ Ry @ Rx

def euler_rate(euler, omega):
    """Compute Euler angle rates from current Euler angles and body angular velocity.
    
    This function uses the transformation for ZYX Euler angles.
    
    Args:
        euler: Euler angles [phi, theta, psi] (radians).
        omega: Angular velocity in body frame [p, q, r] (rad/s).
    
    Returns:
        Euler angle rates [dot_phi, dot_theta, dot_psi] (rad/s).
    """
    phi, theta, _ = euler
    T = np.array([
        [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
        [0, np.cos(phi),              -np.sin(phi)],
        [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]
    ])
    return T @ omega

def wrap_angle(angle):
    """Wrap an angle to the interval [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi

# Simulation loop
for i in range(steps):
    t = i * dt

    # ----- Outer-loop position controller -----
    pos_error = pos_des - pos     # Position error (world frame)
    vel_error = vel_des - vel     # Velocity error (world frame)
    a_cmd = kp_pos * pos_error + kd_pos * vel_error  # Desired acceleration (world frame)
    
    # Compute required force in world frame (F_des = m*(a_cmd - g))
    F_des = mass * (a_cmd - g)
    f_cmd = np.linalg.norm(F_des)  # Thrust magnitude command
    
    # Compute desired roll and pitch angles from F_des (if thrust command is valid)
    if f_cmd > 1e-3:
        # Desired roll: phi_des = atan2(-F_des_y, F_des_z)
        # Desired pitch: theta_des = asin(F_des_x / f_cmd)
        phi_des = np.arctan2(-F_des[1], F_des[2])
        theta_des = np.arcsin(F_des[0] / f_cmd)
    else:
        phi_des = 0.0
        theta_des = 0.0

    # Desired Euler angles: desired roll, pitch and specified yaw
    euler_des = np.array([phi_des, theta_des, psi_des])
    
    # ----- Inner-loop attitude controller -----
    # Compute Euler angle errors (with yaw error wrapped)
    euler_error = np.array([
        euler_des[0] - euler[0],
        euler_des[1] - euler[1],
        wrap_angle(euler_des[2] - euler[2])
    ])
    # PD controller for attitude (roll and pitch) and yaw separately
    torque = np.array([
        kp_att * euler_error[0] - kd_att * omega[0],
        kp_att * euler_error[1] - kd_att * omega[1],
        kp_yaw * euler_error[2] - kd_yaw * omega[2]
    ])
    
    # ----- Dynamics Integration -----
    # Compute current rotation matrix from Euler angles (body-to-world frame)
    R = rotation_matrix(euler)
    # Thrust force in world frame: rotate body thrust [0, 0, f_cmd] into world frame
    F_thrust = R @ np.array([0, 0, f_cmd])
    # Translational dynamics: acceleration in world frame = (F_thrust / m) + g
    acc = F_thrust / mass + g
    vel = vel + acc * dt
    pos = pos + vel * dt

    # Rotational dynamics (Euler's equations for a rigid body)
    omega_dot = I_inv @ (torque - np.cross(omega, I @ omega))
    omega = omega + omega_dot * dt

    # Update Euler angles using computed Euler rates
    euler_dot = euler_rate(euler, omega)
    euler = euler + euler_dot * dt
    euler = np.array([wrap_angle(a) for a in euler])

    # Save state history for plotting
    pos_history.append(pos.copy())
    euler_history.append(euler.copy())
    time_history.append(t)

# Convert histories to numpy arrays for plotting
pos_history = np.array(pos_history)
euler_history = np.array(euler_history)
time_history = np.array(time_history)

# Plot position over time
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(time_history, pos_history[:, 0], label='x')
plt.ylabel('X Position [m]')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time_history, pos_history[:, 1], label='y', color='orange')
plt.ylabel('Y Position [m]')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time_history, pos_history[:, 2], label='z', color='green')
plt.xlabel('Time [s]')
plt.ylabel('Z Position [m]')
plt.legend()

plt.tight_layout()
plt.gcf().canvas.mpl_connect('key_press_event', lambda event: plt.close() if event.key == 'escape' else None)
plt.show()

# Plot Euler angles over time
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(time_history, euler_history[:, 0], label='Roll (phi)')
plt.ylabel('Roll [rad]')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time_history, euler_history[:, 1], label='Pitch (theta)', color='orange')
plt.ylabel('Pitch [rad]')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time_history, euler_history[:, 2], label='Yaw (psi)', color='green')
plt.xlabel('Time [s]')
plt.ylabel('Yaw [rad]')
plt.legend()

plt.tight_layout()
plt.gcf().canvas.mpl_connect('key_press_event', lambda event: plt.close() if event.key == 'escape' else None)
plt.show()
