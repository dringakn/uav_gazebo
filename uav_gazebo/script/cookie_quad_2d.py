#!/usr/bin/env python3
"""
Quadrotor step‐response.
The z-axis is easy to tune, but the y-axis is tricky.
Tune the z-axis first, then the x/y-axis.
The phi gains must be large compared to the x/y/z gains.
Adding integral makes it more complex to tune (required when trajectory vy/vz is non-zero).
"""

import logging
from dataclasses import dataclass
from typing import Callable, Sequence, Tuple, Optional
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# — config dataclasses —
@dataclass(frozen=True)
class Params:
    g: float = 9.81     # m/s²
    m: float = 0.25     # kg
    Ixx: float = 2.5e-4 # kg·m²
    L: float = 0.09     # m
    u_max: float = 2.0 # N

@dataclass(frozen=True)
class Gains:
    Kp_y: float = 0.815
    Kv_y: float = 1.798
    ki_y: float = 0
    Kp_z: float = 10
    Kv_z: float = 10
    ki_z: float = 0
    Kp_phi: float = 72.0
    Kv_phi: float = 60.0
    ki_phi: float = 0

@dataclass(frozen=True)
class AnalyzerConfig:
    rise_start: float = 0.1
    rise_end:   float = 0.9
    settling_pct: float = 0.02
    eps: float = 1e-6
    u_step_eps: float = 1e-3

@dataclass
class State:
    y: float; z: float; phi: float
    vy: float; vz: float; phidot: float

@dataclass
class Desired:
    y: float; z: float
    vy: float; vz: float
    ay: float; az: float

class Controller:
    def __init__(self, kp: float, kv: float, ki: float ):
        self.kp, self.kv, self.ki = kp, kv, ki
        self._err_int, self._last_t = 0.0, None
        self.max_int = 10.0  # tune anti-windup: clamp the integral term
        self.max_dt = 0.1    # max dt for integration (anti-windup)

    def compute(self, err: float, err_dot: float, ff: float = 0.0, t: float = None) -> float:
        if self._last_t is None or t is None: 
            dt = 0
        else:
            dt = min(t - self._last_t, self.max_dt)
        self._last_t = t

        # anti‐windup: only integrate up to max_int
        new_int = self._err_int + err * dt # accumulate area under e(t) i.e ∫e dt term
        self._err_int = max(-self.max_int, min(new_int, self.max_int))        
        
        return ff + self.kp * err + self.kv * err_dot + self.ki * self._err_int

    def reset(self) -> None:
        """Clear integral term and timestamp (e.g. on a new step)."""
        self._err_int = 0.0
        self._last_t = None

class Saturator:
    def __init__(self, minimum: float, maximum: float):
        self.min, self.max = minimum, maximum

    def __call__(self, value: float) -> float:
        sat = np.clip(value, self.min, self.max)
        if sat != value:
            logging.warning("Saturation: raw=%.3f → sat=%.3f", value, sat)
        return sat

class StepResponseAnalyzer:
    def __init__(self, t: np.ndarray, y: np.ndarray, u: np.ndarray,
                 cfg: AnalyzerConfig = AnalyzerConfig()):
        self.cfg = cfg
        self._extract_step(t, y, u)

    def _extract_step(self, t, y, u):
        du = np.diff(u)
        steps = np.where(np.abs(du) > self.cfg.u_step_eps)[0]
        if steps.size:
            i0 = steps[0] + 1
            self.t = t[i0:] - t[i0]
            self.y = y[i0:]
            self.y0 = y[i0]
            self.yf = u[-1]
            self.Δ = self.yf - self.y0
        else:
            self.t, self.y, self.y0, self.yf, self.Δ = t, y, None, None, 0.0

    def compute_metrics(self) -> dict:
        if abs(self.Δ) < self.cfg.eps:
            return {k: None for k in (
                'rise_time','peak_time','overshoot_pct',
                'settling_time','damping_ratio',
                'natural_freq','ss_error')}
        return {
            'rise_time':       self._rise_time(),
            'peak_time':       self._peak_time(),
            'overshoot_pct':   self._overshoot(),
            'settling_time':   self._settling_time(),
            'damping_ratio':   self._damping_ratio(),
            'natural_freq':    self._natural_freq(),
            'ss_error':        self._ss_error(),
        }

    def _rise_time(self) -> Optional[float]:
        t, y, Δ = self.t, self.y, self.Δ
        s = np.sign(Δ)
        y10 = self.y0 + self.cfg.rise_start * Δ
        y90 = self.y0 + self.cfg.rise_end   * Δ
        i1 = np.where(s*(y - y10) >= 0)[0]
        i2 = np.where(s*(y - y90) >= 0)[0]
        if i1.size and i2.size:
            return t[i2[0]] - t[i1[0]]
        return None

    def _peak_time(self) -> float:
        idx = int(np.argmax(np.sign(self.Δ)*(self.y - self.y0)))
        return float(self.t[idx])

    def _overshoot(self) -> float:
        idx = int(np.argmax(np.sign(self.Δ)*(self.y - self.y0)))
        ypk = self.y[idx]
        return float(np.sign(self.Δ)*(ypk - self.yf)/abs(self.Δ)*100)

    def _settling_time(self) -> Optional[float]:
        t, y, Δ = self.t, self.y, self.Δ
        band = self.cfg.settling_pct * abs(Δ)
        out = np.where(np.abs(y - self.yf) > band)[0]
        last = out[-1]+1 if out.size and out[-1]+1 < len(t) else None
        return float(t[last]) if last is not None else None

    def _damping_ratio(self) -> Optional[float]:
        Mp = abs(self._overshoot()/100)
        tp = self._peak_time()
        if Mp and tp:
            return float(-np.log(Mp)/np.sqrt(np.pi**2 + np.log(Mp)**2))
        return None

    def _natural_freq(self) -> Optional[float]:
        ζ = self._damping_ratio()
        tp = self._peak_time()
        if ζ and tp:
            return float(np.pi/(tp * np.sqrt(1 - ζ**2)))
        return None

    def _ss_error(self) -> float:
        tail = max(1, int(0.05*len(self.y)))
        return float(np.mean(self.y[-tail:]) - self.yf)

    @staticmethod
    def annotate(ax, metrics: dict, x=0.05, y=0.95):
        txt = []
        for label, val in metrics.items():
            if val is None: continue
            if 'pct' in label:     fmt = f"{val:.1f}%"
            elif 'time' in label or label=='ss_error':
                fmt = f"{val:.2f}s"
            elif label=='natural_freq':
                fmt = f"{val:.2f} rad/s"
            else:
                fmt = f"{val:.2f}"
            txt.append(f"{label:>12}: {fmt}")
        ax.text(x, y, "\n".join(txt), transform=ax.transAxes,
                verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

class Trajectory:
    def __call__(self, t: float) -> Desired:
        if t >= 1.0:
            return Desired(30, 20, 0, 0, 0, 0)
        return Desired(0, 0, 0, 0, 0, 0)

class Quadrotor:
    def __init__(self, P: Params, G: Gains, 
                 traj: Callable[[float], Desired]):
        self.P, self.traj = P, traj
        # Controller loops
        self.ctrl_y   = Controller(G.Kp_y,   G.Kv_y, G.ki_y)
        self.ctrl_z   = Controller(G.Kp_z,   G.Kv_z, G.ki_z)
        self.ctrl_phi = Controller(G.Kp_phi, G.Kv_phi, G.ki_phi)
        self.sat      = Saturator(0.0, P.u_max)

    def control(self, x: np.ndarray, des: Desired, t: float) -> Tuple[float, float, float]:
        st = State(*x)
        # lateral → φ_cmd
        ay_cmd = self.ctrl_y.compute(des.y - st.y,
                                     des.vy - st.vy,
                                     des.ay, 
                                     t)
        phi_cmd = -ay_cmd / self.P.g
        phi_cmd = max(-np.pi/3, min(phi_cmd, np.pi/3)) # saturate φ_cmd

        # vertical → thrust
        az_cmd = self.ctrl_z.compute(des.z - st.z,
                                     des.vz - st.vz,
                                     des.az,
                                     t)
        F = self.P.m * (self.P.g + az_cmd)

        # attitude torque
        raw_err = phi_cmd - st.phi
        dphi = (raw_err + np.pi) % (2*np.pi) - np.pi # wrap the angle error into [–π, +π)
        M = self.P.Ixx * self.ctrl_phi.compute(dphi, 
                                               -st.phidot,
                                               0,
                                               t)

        # split, saturate, reassemble
        u1 = self.sat(0.5*(F - M/self.P.L))
        u2 = self.sat(0.5*(F + M/self.P.L))
        Fc = u1 + u2
        Mc = (u2 - u1)*self.P.L
        return Fc, Mc, phi_cmd

    def dynamics(self, t: float, x: np.ndarray) -> np.ndarray:
        des = self.traj(t)
        Fc, Mc, _ = self.control(x, des, t)
        ydot, zdot, phidot = x[3], x[4], x[5]
        ay    = -Fc * np.sin(x[2]) / self.P.m
        az    =  Fc * np.cos(x[2]) / self.P.m - self.P.g
        phidd = Mc / self.P.Ixx
        return np.array([ydot, zdot, phidot, ay, az, phidd])

class Simulator:
    def __init__(self, quad: Quadrotor, 
                 x0: Sequence[float], t_final: float):
        self.quad, self.x0, self.t_final = quad, x0, t_final

    def run(self):
        self.quad.ctrl_y.reset()
        self.quad.ctrl_z.reset()
        self.quad.ctrl_phi.reset()
        return solve_ivp(self.quad.dynamics,
                         (0, self.t_final), self.x0,
                         method='RK45', rtol=1e-6, atol=1e-9,
                         max_step=0.01)

class Plotter:
    def __init__(self, quad: Quadrotor):
        self.quad = quad

    def plot(self, sol):
        t, Y = sol.t, sol.y
        des = [self.quad.traj(tt) for tt in t]

        # commanded trajectories
        u_y     = np.array([d.y for d in des])
        u_z     = np.array([d.z for d in des])
        u_vy  = np.array([d.vy for d in des])    # <-- desired y‐rate
        u_vz  = np.array([d.vz for d in des])    # <-- desired z‐rate

        # commanded roll φ (deg) and its rate (deg/s)
        phi_cmd_deg = np.rad2deg([
            self.quad.control(Y[:,i], des[i], t[i])[2]
            for i in range(len(t))
        ])
        phidot_deg = np.rad2deg(Y[5])

        # control actions
        Fc = np.empty_like(t); Mc = np.empty_like(t)
        for i, tt in enumerate(t):
            Fc[i], Mc[i], _ = self.quad.control(Y[:,i], des[i], tt)

        #--- layout with GridSpec ---#
        fig = plt.figure(figsize=(10,12), constrained_layout=True)
        gs  = fig.add_gridspec(4, 2, hspace=0.4, wspace=0.3)
        
        # Row 0
        ax1 = fig.add_subplot(gs[0,0])
        ax2 = fig.add_subplot(gs[0,1])
        # Row 1
        ax3 = fig.add_subplot(gs[1,0])
        ax4 = fig.add_subplot(gs[1,1])
        # Row 2
        ax5 = fig.add_subplot(gs[2,0])
        ax6 = fig.add_subplot(gs[2,1])
        # Row 3 spans both columns
        ax7 = fig.add_subplot(gs[3, :])

        # y / ẏ
        ax1.plot(t, Y[0],    label='y actual');   ax1.plot(t, u_y, '--', label='y cmd')
        StepResponseAnalyzer(t, Y[0], u_y).compute_metrics(); 
        StepResponseAnalyzer.annotate(ax1, StepResponseAnalyzer(t,Y[0],u_y).compute_metrics())
        ax1.set_ylabel('y (m)'); ax1.legend(loc='best'); ax1.grid(True)

        ax2.plot(t, Y[3],    label='ẏ actual')
        ax2.plot(t, u_vy, '--', label='ẏ cmd')    # <— reference
        ax2.set_ylabel('ẏ (m/s)'); ax2.legend(loc='best'); ax2.grid(True)
        
        # z / ż
        ax3.plot(t, Y[1],    label='z actual');   ax3.plot(t, u_z, '--', label='z cmd')
        StepResponseAnalyzer.annotate(ax3, StepResponseAnalyzer(t,Y[1],u_z).compute_metrics())
        ax3.set_ylabel('z (m)'); ax3.legend(loc='best'); ax3.grid(True)

        ax4.plot(t, Y[4],    label='ż actual')
        ax4.plot(t, u_vz, '--', label='ż cmd')    # <— reference
        ax4.set_ylabel('ż (m/s)'); ax4.legend(loc='best'); ax4.grid(True)

        # φ / φ̇
        ax5.plot(t, np.rad2deg(Y[2]), label='φ actual')
        ax5.plot(t, phi_cmd_deg,      '--', label='φ cmd')
        ax5.text(0.05, 0.95, "φ: no valid step metrics",
                 transform=ax5.transAxes, va='top',
                 bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        ax5.set_ylabel('φ (°)'); ax5.legend(loc='best'); ax5.grid(True)

        ax6.plot(t, phidot_deg, label='φ̇'); ax6.set_ylabel('φ̇ (°/s)'); ax6.grid(True)

        # control actions spanning both cols
        ax7.plot(t, Fc, '-',  label='Fc (thrust)')
        ax7_twin = ax7.twinx()
        ax7_twin.plot(t, Mc, 'r--', label='Mc (moment)')
        ax7.set_ylabel('Thrust (N)'); ax7_twin.set_ylabel('Moment (N·m)')
        lines, labs = ax7.get_legend_handles_labels()
        lines2, labs2 = ax7_twin.get_legend_handles_labels()
        ax7.legend(lines+lines2, labs+labs2, loc='best')
        ax7.set_xlabel('Time (s)'); ax7.grid(True)

        # ensure clean layout and quick close
        plt.connect('key_press_event',
                    lambda e: plt.close() if e.key == 'escape' else None)
        plt.show()
        
def main():
    logging.basicConfig(level=logging.ERROR,
                        format="%(levelname)s: %(message)s")
    quad = Quadrotor(Params(), Gains(), Trajectory())
    sim  = Simulator(quad, x0=[0.]*6, t_final=50.0)
    sol  = sim.run()
    Plotter(quad).plot(sol)

if __name__ == "__main__":
    main()
