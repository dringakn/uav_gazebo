#!/usr/bin/env python3
"""
Author: Dr. -Ing. Ahmad Kamal Nasir <dringakn@gmail.com>

Quadrotor Simulator — Parameters Module
---------------------------------------
This module centralizes all of the core physical constants for the quadrotor
model. By encapsulating mass, arm length, inertia, and gravity here in an immutable
dataclass, we achieve:

  • Consistency: every part of the simulator pulls from the same source of truth  
  • Maintainability: updating vehicle specs or running parameter sweeps is trivial  
  • Safety: frozen dataclass prevents accidental runtime modifications  
  • Clarity: self-documenting code that clearly separates model constants  
    from algorithms and control logic

Classes:
  • QuadParams: frozen container holding  
      - mass   [kg]  
      - Inertia tensor (body frame) [kg·m²]  
      - Gravity vector (world frame) [m/s²]

Usage:
  from params import QuadParams
  params = QuadParams()
  print(params.mass, params.I, params.g)
"""

from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class QuadParams:
    """
    Immutable physical constants for the 250g quadrotor.
    Many 250 g “180 mm” class quads use ~30 g brushless motors and a ~90 mm arm‐to‐arm radius.
    Point‐mass rotor model: Each rotor (≈ 0.03 kg) at radius r≈0.09 m contributes I = m r² per axis. 
    Two rotors lie on each principal axis (roll/pitch), 
    four rotors spin about the yaw axis.
    Estimate 4 rotors as point-masses at r≈0.09 m from CM:
        I = Σ m_i r²  →  Izz = 4·0.03·0.09² ≈ 1.0e-3 [kg·m²]
                         Ixx=Iyy = 2·0.03·0.09² ≈ 5.0e-4 [kg·m²]
    These are first-order estimates; 
    for higher fidelity you can add a small central‐body term 
    (e.g. solid-disk or rectangular‐plate inertia via 1/12 m(a²+b²)) 
    or do a pendulum/CAD analysis.
    """    
    mass    = 0.25        # total [kg]
    m_rotor = 0.03        # each rotor [kg]
    m_body  = mass - 4*m_rotor  # ≈0.13 kg

    # geometry
    r_rot   = 0.09        # rotor arm length [m]
    r_body  = 0.07        # body radius [m]
    h_body  = 0.005       # body thickness [m]

    # rotor inertia contributions
    I_rot_xy = 2*m_rotor*r_rot**2    # two rotors per principal axis
    I_rot_z  = 4*m_rotor*r_rot**2    # all four about yaw

    # body (solid disk) inertia
    I_body_xy = 1/4*m_body*r_body**2 + 1/12*m_body*h_body**2
    I_body_z  = 1/2*m_body*r_body**2

    # total inertia ≈ [6.5e-4, 6.5e-4, 1.29e-3]
    I = np.diag([
        I_rot_xy + I_body_xy, 
        I_rot_xy + I_body_xy, 
        I_rot_z  + I_body_z
    ])

    g = np.array([0.0, 0.0, -9.81])