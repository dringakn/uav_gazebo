#!/usr/bin/env python3
"""
Author: Dr. -Ing. Ahmad Kamal Nasir <dringakn@gmail.com>

Quadrotor Simulator — Parameters Module
---------------------------------------
This module centralizes all of the core physical constants for the quadrotor
model. By encapsulating mass, inertia, and gravity here in an immutable
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
    """Immutable physical constants for the quadrotor."""
    mass: float = 1.0
    I:    np.ndarray = np.diag([0.003, 0.003, 0.006])  # Inertia [kg·m²] in body frame
    g:    np.ndarray = np.array([0.0, 0.0, -9.81])     # Gravity [m/s²] in world frame
