#!/usr/bin/env python3
"""
Author: Dr. -Ing. Ahmad Kamal Nasir <dringakn@gmail.com>

Module: controller.py

This module implements a cascaded PD controller for a six‐DOF quadrotor UAV.
It separates control into two nested loops:

  1. Outer‐loop Position Control:
     - Computes position error and velocity error in the world frame.
     - Applies PD gains (kp_pos, kd_pos) to generate a desired acceleration.
     - Converts that into a desired thrust vector F_des = m·(a_cmd − g).

  2. Inner‐loop Attitude Control on SO(3):
     - Constructs the desired rotation R_des by aligning the body‐z axis
       with F_des and enforcing a commanded yaw angle.
     - Extracts the minimal rotation error e_R = ½ (R_desᵀ R_cur − R_curᵀ R_des)∨
       as in Lee et al. (2010).
     - Applies PD gains (kp_att, kd_att) on e_R and body‐rate damping on ω
       to compute control torques τ.

The final control output is a 4×1 vector [F_z, τ_x, τ_y, τ_z] in the body frame,
ready to be fed into the vehicle dynamics integrator.

References:
    T. Lee, N. Michael, and S. S. Sastry, “Geometric control of quadrotor UAVs”,
    in *Proc. IEEE CDC*, 2010.
"""

import numpy as np
from typing import Tuple
from params import QuadParams
from state import QuadState
from math_utils import quat_to_rot
from dataclasses import dataclass

@dataclass
class PDGains:
    kp_pos:  float = 0.25
    kd_pos:  float = 1.0
    kp_att:  float = 25.0
    kd_att:  float = 10.0
    kp_yaw:  float = 2.25
    kd_yaw:  float = 3.0

class CascadedPD:
    """Outer-loop position PD → desired R, inner-loop SO(3) PD."""

    def __init__(self, p: QuadParams, gains: PDGains):
        self.p, self.g = p, gains

    # ------------------------------------------------------------------
    def __call__(
        self,
        t: float,
        s: QuadState,
        pos_ref: np.ndarray,
        vel_ref: np.ndarray,
        yaw_ref: float,
    ) -> np.ndarray:
        """Return control vector u = [Fz, τx, τy, τz]."""

        # ----- position loop ------------------------------------------
        pos_err = pos_ref - s.pos
        vel_err = vel_ref - s.vel
        a_cmd   = self.g.kp_pos*pos_err + self.g.kd_pos*vel_err
        F_des_w = self.p.mass*(a_cmd - self.p.g)

        # Desired body-Z axis
        b3_des = F_des_w/np.linalg.norm(F_des_w)

        # Construct desired rotation from yaw
        c_yaw = np.array([np.cos(yaw_ref), np.sin(yaw_ref), 0])
        b2_des = np.cross(b3_des, c_yaw); b2_des /= np.linalg.norm(b2_des)
        b1_des = np.cross(b2_des, b3_des)
        R_des  = np.column_stack((b1_des, b2_des, b3_des))

        # Current rotation
        R_cur  = quat_to_rot(s.quat)

        # SO(3) attitude error (Lee et al. 2010)
        err_Rmat  = 0.5*(R_des.T @ R_cur - R_cur.T @ R_des)
        err_R     = np.array([err_Rmat[2,1], err_Rmat[0,2], err_Rmat[1,0]])

        # Yaw rate error (des = 0)
        tau = ( self.g.kp_att*err_R
              - self.g.kd_att*s.omega )

        # Desired thrust magnitude in body frame (+Z)
        Fz_body = (R_des.T @ F_des_w)[2]

        return np.hstack([Fz_body, tau])
