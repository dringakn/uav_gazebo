#!/usr/bin/env python3
"""
Author: Dr. -Ing. Ahmad Kamal Nasir <dringakn@gmail.com>

Module: QuadDynamics

Description:
    Implements the full six‐degree‐of‐freedom rigid‐body equations of motion
    for a quadrotor UAV using quaternion‐based attitude representation.
    Given body‐frame thrust and torques, it computes world‐frame translational
    accelerations and body‐frame angular accelerations via:

      • Translational dynamics (in world frame):
          m·d²p/dt² = m·g + R(q)·[0, 0, F_z]ᵀ
        where:
          - p ∈ ℝ³ is the position vector,
          - m is the vehicle mass,
          - g = [0, 0, −9.81]ᵀ m/s²,
          - F_z is total thrust along the body z‐axis,
          - R(q) ∈ SO(3) is the rotation matrix from body to world, derived from quaternion q.

      • Rotational dynamics (in body frame):
          I·dω/dt = τ − ω × (I·ω)
        where:
          - ω ∈ ℝ³ is the angular velocity in the body frame,
          - I ∈ ℝ³ˣ³ is the inertia matrix (body‐fixed),
          - τ ∈ ℝ³ is the control torque vector,
          - × denotes the vector cross‐product.

      • Quaternion kinematics:
          dq/dt = ½ · q ⊗ [ω, 0]
        using the Hamilton product ⊗, which propagates orientation without
        suffering from Euler‐angle singularities.

    Interfaces provided:
      - rhs(s: QuadState, u: ndarray) → QuadState
          Compute continuous‐time derivatives of the QuadState given control
          inputs u = [F_z_body, τ_x, τ_y, τ_z].
      - rhs_vec(x: ndarray, u: ndarray) → ndarray
          Flattened‐vector wrapper for use with generic ODE solvers
          (e.g. RK4, SciPy solve_ivp).

Dependencies:
    numpy
    params.QuadParams
    state.QuadState
    math_utils.quat_to_rot
    math_utils.quat_deriv
"""
import numpy as np
from params import QuadParams
from state import QuadState
from math_utils import quat_to_rot, quat_deriv

class QuadDynamics:
    """Rigid-body six-DOF + thrust/torque inputs."""

    def __init__(self, p: QuadParams):
        self.p = p
        self.I_inv = np.linalg.inv(p.I)

    def rhs(self, s: QuadState, u: np.ndarray) -> QuadState:
        """Compute continuous‐time derivatives of the QuadState given control.

        Args:
            s: current state (pos, vel, quat, omega)
            u: control inputs [F_z_body, τ_x, τ_y, τ_z]
        Returns:
            QuadState of derivatives (pos_dot, vel_dot, quat_dot, omega_dot)
        """
        # Decompose thrust magnitude and body‐frame torques
        thrust, tau = u[0], u[1:]  

        # 1) Compute body→world rotation matrix from quaternion
        R_bw = quat_to_rot(s.quat)

        # 2) Map thrust vector [0, 0, thrust]₍B₎ into world frame
        F_thrust_w = R_bw @ np.array([0.0, 0.0, thrust])

        # 3) Kinematics: position rate is current velocity
        pos_dot = s.vel

        # 4) Translational dynamics: a = g + F/m
        vel_dot = self.p.g + (F_thrust_w / self.p.mass)

        # 5) Rotational dynamics: I·ω̇ = τ − ω×(I·ω)
        #    compute ω̇ = I⁻¹ (τ − ω×(I·ω))
        omega_dot = self.I_inv @ (tau - np.cross(s.omega, self.p.I @ s.omega))

        # 6) Quaternion kinematics: q̇ = ½ · q ⊗ [ω, 0]
        quat_dot = quat_deriv(s.quat, s.omega)

        return QuadState(pos_dot, vel_dot, quat_dot, omega_dot)

    def rhs_vec(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Vectorized wrapper for use with generic ODE integrators.
        
        Args:
            x (np.ndarray): Flattened state vector of length 13,
                [pos(3), vel(3), quat(4), omega(3)].
            u (np.ndarray): Control input vector [F_z_body, τ_x, τ_y, τ_z].
        
        Returns:
            np.ndarray: Flattened derivative vector of the same shape as x.
        """
        # 1. Reconstruct the rich-state object from the flat vector:
        #    - pos   = x[0:3]
        #    - vel   = x[3:6]
        #    - quat  = x[6:10]
        #    - omega = x[10:13]
        state = QuadState.from_vec(x)

        # 2. Compute the continuous-time derivatives for each component:
        #    - pos_dot   = current velocity
        #    - vel_dot   = acceleration from thrust and gravity
        #    - quat_dot  = quaternion rate from angular velocity
        #    - omega_dot = angular acceleration from body torques
        state_dot = self.rhs(state, u)

        # 3. Flatten the derivative QuadState back into a single vector:
        #    Concatenates [pos_dot, vel_dot, quat_dot, omega_dot]
        x_dot = state_dot.as_vec()

        # 4. Return the flat derivative vector for the integrator
        return x_dot

