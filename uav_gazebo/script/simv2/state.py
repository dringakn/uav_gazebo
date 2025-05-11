#!/usr/bin/env python3
"""
Author: Dr. -Ing. Ahmad Kamal Nasir <dringakn@gmail.com>

state.py — QuadState dataclass and utilities for continuous‐time quadrotor simulation

This module defines the core state representation for a six‐degree‐of‐freedom
quadrotor model. It encapsulates position, velocity, attitude (as a unit quaternion),
and angular velocity in a single immutable dataclass, and provides conversion
routines to and from flat NumPy vectors for use with integrators.

Key components:
  - QuadState.pos   : 3-element NumPy array; position in the world (in meters)
  - QuadState.vel   : 3-element NumPy array; linear velocity in the world (m/s)
  - QuadState.quat  : 4-element NumPy array; orientation as (x, y, z, w) quaternion
                       mapping body→world
  - QuadState.omega : 3-element NumPy array; angular rate in the body frame (rad/s)

Helper methods:
  - copy()       : deep-copy the state to avoid accidental aliasing
  - __iter__()   : allow tuple-unpacking (pos, vel, quat, omega)
  - as_vec()     : pack the state into a (13,) vector [pos; vel; quat; omega]
  - from_vec(v)  : reconstruct a QuadState from a (13,) NumPy array

Usage:
  * Instantiate a fresh state at time t=0:
        s0 = QuadState(
            pos  = np.zeros(3),
            vel  = np.zeros(3),
            quat = np.array([0.0, 0.0, 0.0, 1.0]),
            omega= np.zeros(3)
        )
  * Pass s0.as_vec() to an integrator.rhs_vec callback.
  * Recover the state via QuadState.from_vec(x) each time‐step.

This clean separation of data and dynamics enables:
  - Modular integrators (RK4, SciPy solve_ivp, custom solvers)
  - Unit testing of pure‐function controllers and dynamics
  - Easy logging and plotting of individual state components

Part of a larger quadrotor simulation framework:
  params.py      — immutable physical parameters
  dynamics.py    — continuous‐time rigid‐body equations
  controller.py  — cascaded PD control laws
  integrator.py  — generic RK4 integrator
  simulate.py    — high-level experiment runner + plotting
"""

from dataclasses import dataclass
import numpy as np
from typing import Tuple

@dataclass
class QuadState:
    """Continuous state vector."""
    pos:   np.ndarray        # (3,) world  [m]
    vel:   np.ndarray        # (3,) world  [m s⁻¹]
    quat:  np.ndarray        # (4,) (x,y,z,w) scalar-last
    omega: np.ndarray        # (3,) body   [rad s⁻¹]

    # --- helpers -----------------------------------------------------------
    def copy(self) -> "QuadState":
        """Return a deep copy of the state."""
        return QuadState(*(x.copy() for x in self))

    def __iter__(self):
        """Allow unpacking: pos, vel, quat, omega = state."""
        yield self.pos
        yield self.vel
        yield self.quat
        yield self.omega

    def as_vec(self) -> np.ndarray:
        """Pack state into a flat vector [pos; vel; quat; omega]."""
        return np.hstack([self.pos, self.vel, self.quat, self.omega])

    @staticmethod
    def from_vec(v: np.ndarray) -> "QuadState":
        """Reconstruct a QuadState from a (13,) vector."""
        return QuadState(
            pos   = v[0:3],
            vel   = v[3:6],
            quat  = v[6:10],
            omega = v[10:13]
        )
