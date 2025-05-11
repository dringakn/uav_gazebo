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

    # ------------------------------------------------------------------
    def rhs(self, s: QuadState, u: np.ndarray) -> QuadState:
        """Continuous-time state derivatives.

        u = [ F_z_body, τ_x, τ_y, τ_z ]  (F along body +Z)
        """
        thrust, tau = u[0], u[1:]

        R_bw = quat_to_rot(s.quat)        # body→world
        F_thrust_w = R_bw @ np.array([0, 0, thrust])

        pos_dot = s.vel
        vel_dot = F_thrust_w / self.p.mass + self.p.g

        omega_dot = self.I_inv @ (tau - np.cross(s.omega, self.p.I @ s.omega))
        quat_dot  = quat_deriv(s.quat, s.omega)

        return QuadState(pos_dot, vel_dot, quat_dot, omega_dot)

    # ------------------------------------------------------------------
    # Convenience for vector-based integrator
    def rhs_vec(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        return self.rhs(QuadState.from_vec(x), u).as_vec()
