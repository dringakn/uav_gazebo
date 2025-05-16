#!/usr/bin/env python3
"""
Author: Dr. -Ing. Ahmad Kamal Nasir <dringakn@gmail.com>

Quaternion Utilities for Rigid‐Body Attitude Kinematics and Control
===================================================================

This module provides core quaternion operations using the scalar‐last
convention (x, y, z, w), tailored for 3D rigid‐body simulations
(e.g., quadrotor UAVs). It includes:

  • quat_to_rot(q):
      Convert a unit quaternion q into a 3×3 rotation matrix R ∈ SO(3).
      Relies on SciPy’s Rotation class for numerical robustness.

  • quat_multiply(q1, q2):
      Compute the Hamilton product (q1 ⊗ q2), enabling quaternion‐based
      composition of rotations without gimbal‐lock.

  • quat_deriv(q, omega):
      Compute the time derivative of q given body‐frame angular velocity
      ω = [p, q, r]. Implements q̇ = ½·q⊗[ω, 0], critical for integrating
      attitude over time using standard ODE solvers.

These helpers form the backbone of attitude representation, error
evaluation, and integration in feedback controllers and dynamics
simulators. All inputs and outputs are NumPy arrays for seamless
interoperability with NumPy/SciPy ecosystems.

References:
  – Hamilton, W. R. (1844). On Quaternions; or on a new System of Imaginaries.
  – Kuipers, J. B. (1999). Quaternions and Rotation Sequences.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R

# ---------------------------------------------------------------------
# Quaternion helpers (scalar-last convention: (x, y, z, w))
# ---------------------------------------------------------------------

def quat_to_rot(q: np.ndarray) -> np.ndarray:
    return R.from_quat(q).as_matrix()

def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product (both scalar-last)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])

def quat_deriv(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    """q̇ = 0.5 · q ⊗ ω̂,  ω̂ = (ω,0)."""
    return 0.5 * quat_multiply(q, np.hstack([omega, 0.0]))

def safe_normalise(q: np.ndarray) -> np.ndarray:
    """Return q/‖q‖ – but if ‖q‖≈0 fall back to identity (0,0,0,1)."""
    n = np.linalg.norm(q)
    if n < 1e-8 or not np.isfinite(n):
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n