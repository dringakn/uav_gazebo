# Quadrotor Simulator

A modular, extensible Python framework for six-DOF quadrotor simulation with cascaded PD control.  
Built on NumPy and SciPy, it cleanly separates physics, control, integration, and I/O so you can swap in new models, controllers or integrators without touching the core.

---

## Features

- **Six-DOF rigid‐body dynamics** with thrust & torque inputs  
- **Cascaded PD controller**: outer‐loop position, inner‐loop attitude  
- **Quaternion-based attitude** to avoid gimbal lock  
- **Pluggable integrators** (RK4 by default; swap in `solve_ivp`, GPU kernels, etc.)  
- **Immutable parameter set** (`QuadParams`) for easy tuning & config files  
- **Headless logging + standalone plotting** via Matplotlib  

---

## Requirements

- Python 3.8+  
- NumPy  
- SciPy  
- Matplotlib  

Install via:

```bash
pip install numpy scipy matplotlib
````

---

## Installation

Clone the repo and add to your `PYTHONPATH`, or install in editable mode:

```bash
git clone https://github.com/dringakn/uav_gazebo.git
cd uav_gazebo/uav_gazebo/scripts/simv2
pip install -e .
```

---

## Quickstart

```bash
python simulate.py
```

This flies a 10 s hover at (0, 0, 5 m) and plots the position over time.

---

## Project Layout

```
simv2
├── params.py        # physical constants & gravity
├── state.py         # QuadState dataclass (pos, vel, quat, omega)
├── math_utils.py    # quaternion ↔ rotation, kinematics
├── dynamics.py      # continuous‐time rigid‐body RHS
├── controller.py    # cascaded PD: position → attitude → thrust/torque
├── integrator.py    # generic RK4 integrator
├── simulate.py      # example runner + plotting
└── README.md        # this file
```

---

## Mathematical Model

### Reference Frames

* **World frame** $\{W\}$: inertial frame; position $\mathbf{p}$ and velocity $\mathbf{v}$ live here.
* **Body frame** $\{B\}$: attached to quadrotor; thrust acts along the body-z axis.

### State Variables

$$
\mathbf{s} = \bigl(\mathbf{p},\;\mathbf{v},\;\mathbf{q},\;\boldsymbol\omega\bigr)
$$

* $\mathbf{p}\in\mathbb{R}^3$: position in $\{W\}$.
* $\mathbf{v}\in\mathbb{R}^3$: velocity in $\{W\}$.
* $\mathbf{q}\in\mathbb{R}^4$: unit quaternion $(x,y,z,w)$ rotating $\{B\}\to\{W\}$.
* $\boldsymbol\omega\in\mathbb{R}^3$: angular rate in $\{B\}$.

### Translational Dynamics

$$
m\,\ddot{\mathbf{p}}
= m\,\mathbf{g} \;+\; \mathbf{R}(\mathbf{q})\,\begin{bmatrix}0\\0\\F_z\end{bmatrix}
\quad\Longrightarrow\quad
\dot{\mathbf{v}}
= \mathbf{g} + \frac{1}{m}\,R(\mathbf{q})\,\begin{bmatrix}0\\0\\F_z\end{bmatrix},
$$

where

* $m$ is mass,
* $\mathbf{g}=[0,0,-9.81]^T$ m/s²,
* $F_z$ is the total body-frame thrust,
* $R(\mathbf{q})\in\mathrm{SO}(3)$ converts body→world.

### Rotational Dynamics

$$
\mathbf{I}\,\dot{\boldsymbol\omega}
= \boldsymbol\tau \;-\;\boldsymbol\omega\times(\mathbf{I}\,\boldsymbol\omega),
$$

with inertia $\mathbf{I}=\mathrm{diag}(I_x,I_y,I_z)$ and control torque $\boldsymbol\tau$.

### Quaternion Kinematics

$$
\dot{\mathbf{q}} \;=\; \tfrac12\,\mathbf{q}\,\otimes\,[\omega_x,\omega_y,\omega_z,0],
$$

using Hamilton product $\otimes$; keeps $\mathbf{q}$ unit-norm.

---

## Control Architecture

1. **Outer‐loop (Position PD)**

   $$
     \mathbf{a}_\text{cmd}
     = k_p(\mathbf{p}_\text{ref}-\mathbf{p})
       + k_d(\mathbf{v}_\text{ref}-\mathbf{v})
   \;\;\Longrightarrow\;\;
     \mathbf{F}_\text{des}
     = m\bigl(\mathbf{a}_\text{cmd}-\mathbf{g}\bigr)
   $$
2. **Attitude set-point**
   Align body-z with $\mathbf{F}_\text{des}$, fix yaw to $\psi_\text{ref}$.
3. **Inner-loop (SO(3) PD)**
   Compute rotation error $e_R\in\mathbb{R}^3$ from $R_\text{des}^T R$, then
   $\boldsymbol\tau = k_R\,e_R - k_\omega\,\boldsymbol\omega$.

---

## Extending & Testing

* **New integrator?** Swap in SciPy’s `solve_ivp` by replacing the `RK4` instance.
* **Multiple agents?** Create multiple `QuadState` vectors and step them in parallel.
* **Unit tests**: Controllers and dynamics are pure functions; write pytest cases on `rhs` and `__call__`.

---

## License

MIT © Dr. -Ing. Ahmad Kamal Nasir

