#!/usr/bin/env python3

from vpython import *
import numpy as np

# Simulation parameters
dt = 0.05          # Time step (s)
T = 20.0           # Total simulation time (s)
t = 0.0

# Rotor model parameters (realistic values)
turning_direction = 1
rotor_velocity_slowdown_sim = 10.0
motor_constant = 2e-5              # Thrust scaling constant [N/(rad/s)^2]
rotor_drag_coefficient = 4e-4      # Drag coefficient [N/(m/s)/rad/s]
moment_constant = 0.01             # Yaw moment constant [N*m per N of thrust]
rolling_moment_coefficient = 2e-5  # Rolling moment constant [N*m/(m/s)/rad/s]
time_constant_up = 0.2             # Filter time constant when speeding up (s)
time_constant_down = 0.3           # Filter time constant when slowing down (s)

# Constant forward body velocity (for drag and moments)
body_velocity = vector(5, 0, 0)  # m/s

# Initialize rotor angular velocity (state for filtering)
omega = 0.0

# Rotor speed command (sinusoidal variation around 50 rad/s)
def rotor_speed_command(t):
    return 50.0 + 10.0 * np.sin(0.5 * t)

# Set up the VPython scene
scene = canvas(title="Rotor Dynamics Forces and Torques",
               width=800, height=600,
               center=vector(0, 0, 0),
               background=color.gray(0.2))
scene.camera.pos = vector(0, -10, 5)
scene.camera.axis = vector(0, 10, -5)

# Visual representation of the rotor: a white ring (rotor disk) of radius 0.3 m
rotor_disk = ring(pos=vector(0, 0, 0), axis=vector(0, 0, 1),
                  radius=0.3, thickness=0.02, color=color.white)

# Create arrows with distinct base positions for clarity:
# - Thrust arrow (green): from rotor center upward.
# - Drag arrow (red): from below the rotor.
# - Yaw torque arrow (cyan): to the right of the rotor.
# - Rolling moment arrow (magenta): to the left of the rotor.
arrow_thrust = arrow(pos=vector(0,0,0), axis=vector(0,0,0),
                     color=color.green, shaftwidth=0.04)
arrow_drag   = arrow(pos=vector(0,-1,0), axis=vector(0,0,0),
                     color=color.red, shaftwidth=0.04)
arrow_yaw    = arrow(pos=vector(1,0,0), axis=vector(0,0,0),
                     color=color.cyan, shaftwidth=0.04)
arrow_roll   = arrow(pos=vector(-1,0,0), axis=vector(0,0,0),
                     color=color.magenta, shaftwidth=0.04)

# Label to display simulation information
info_label = label(pos=vector(0,0,2), text="", height=16, box=False)

# Scaling factors for visualization (adjust these so the arrows appear proportionate)
thrust_scale = 0.25   # Thrust arrow scale
drag_scale   = 1.0    # Drag arrow scale
torque_scale = 50.0   # Torque arrow scale

# Simulation loop
while t < T:
    rate(1/dt)  # Controls simulation speed
    
    # Update rotor speed using a first-order filter
    omega_ref = rotor_speed_command(t)
    tau = time_constant_up if omega_ref > omega else time_constant_down
    omega += (omega_ref - omega) / tau * dt
    
    # Compute effective rotor speed
    omega_real = omega * rotor_velocity_slowdown_sim
    
    # 1. Thrust force: F = (ω_real)^2 * motor_constant,
    # with a reduction factor based on forward velocity.
    F_thrust = (omega_real**2) * motor_constant
    scalar = max(0.0, min(1.0, 1 - mag(body_velocity) / 25.0))
    F_thrust_eff = F_thrust * scalar
    thrust_vec = vector(0, 0, F_thrust_eff)
    
    # 2. Aerodynamic drag force: acting opposite to the forward velocity.
    rotor_axis = vector(0, 0, 1)
    body_velocity_perp = body_velocity - (dot(body_velocity, rotor_axis))*rotor_axis
    F_drag = -abs(omega_real) * rotor_drag_coefficient * body_velocity_perp
    
    # 3. Yaw torque (drag torque): about the vertical (z) axis.
    tau_yaw = -turning_direction * F_thrust * moment_constant
    yaw_vec = vector(0, 0, tau_yaw)
    
    # 4. Rolling moment: computed similarly to drag (about the x axis in this case).
    rolling_vec = -abs(omega_real) * rolling_moment_coefficient * body_velocity_perp
    
    # Update arrow axes with the computed forces/torques (scaled for visibility)
    arrow_thrust.axis = thrust_vec * thrust_scale
    arrow_drag.axis   = F_drag * drag_scale
    arrow_yaw.axis    = yaw_vec * torque_scale
    arrow_roll.axis   = rolling_vec * torque_scale
    
    # Update simulation info text
    info_label.text = (f"t = {t:.2f} s\nω = {omega:.2f} rad/s\n"
                       f"ω_ref = {omega_ref:.2f} rad/s\nThrust: {F_thrust_eff:.2f} N")
    
    t += dt
