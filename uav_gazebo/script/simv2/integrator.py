#!/usr/bin/env python3
"""
Author: Dr. -Ing. Ahmad Kamal Nasir <dringakn@gmail.com>

Module: RK4 Integrator
----------------------

This module implements a generic fourth‐order Runge–Kutta (RK4) solver for
autonomous or controlled ordinary differential equations on ℝⁿ of the form:

    ˙x = f(x, u)

where x ∈ ℝⁿ is the state vector and u ∈ ℝᵐ is an optional input or control
vector.  

Classes:
    RK4
        Encapsulates the RK4 integration scheme. Users supply a function
        f(x, u) that computes time derivatives, and then call .step(x, u, dt)
        to advance the state by a fixed timestep dt.

Usage Example:
    ```python
    import numpy as np
    from rk4 import RK4

    # Define dynamics: simple linear system ˙x = A x + B u
    A = np.array([[0, 1], [-1, 0]])
    B = np.eye(2)
    def f(x, u):
        return A @ x + B @ u

    integrator = RK4(f)
    x0 = np.array([1.0, 0.0])
    u0 = np.array([0.0, 0.0])
    dt = 0.01

    x1 = integrator.step(x0, u0, dt)
    ```

Dependencies:
    - numpy

"""

import numpy as np
from typing import Callable
from math_utils import safe_normalise

class RK4:
    """Generic fourth‐order Runge–Kutta integrator on ℝⁿ."""

    def __init__(self, f: Callable[[np.ndarray, np.ndarray], np.ndarray]):
        """
        Initialize the RK4 integrator.

        Args:
            f: A function f(x, u) → ẋ that returns the time derivative of the
               state given the current state x and input u.
        """
        self.f = f

    def _renorm(self, x: np.ndarray) -> None:
        """In-place quaternion renormalisation (indices 6:10)."""
        x[6:10] = safe_normalise(x[6:10])
        
    def step(self, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
        """
        Advance the state by one time step using RK4.

        Args:
            x:  Current state vector (n,).
            u:  Current input/control vector (m,).
            dt: Time step to integrate over.

        Notes:
            If you only normalize q in dynamics.rhs, 
            you’ll still see overflows or NaNs in the integrator’s 
            temporary states. Keeping normalization in the integrator 
            is the cleanest, safest way to enforce ‖q‖=1.

        Returns:
            x_next: State vector after time dt.
        """
        # k1
        k1 = self.f(x, u)

        # k2
        x2 = x + 0.5*dt*k1
        self._renorm(x2)
        k2 = self.f(x2, u)

        # k3
        x3 = x + 0.5*dt*k2
        self._renorm(x3)
        k3 = self.f(x3, u)

        # k4
        x4 = x + dt*k3
        self._renorm(x4)
        k4 = self.f(x4, u)

        x_next = x + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)
        self._renorm(x_next)
        
        return x_next
