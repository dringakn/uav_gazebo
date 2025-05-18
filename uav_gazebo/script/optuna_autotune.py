#!/usr/bin/env python3
import optuna
import numpy as np
import logging
from dataclasses import replace

from cookie_quad_2d import Params, Gains, Trajectory, Quadrotor, Simulator, StepResponseAnalyzer

logging.getLogger().setLevel(logging.ERROR)

# total sim time
t_final = 10.0
overshoot_weight = 10.0 # ↑ more emphasis on overshoot
settling_weight = 100.0   # ↑ more emphasis on settling

# define any axes you want tune
axes = {
    'y': {
        'idx': 0,           # sol.y index
        'cmd': 'y',         # Trajectory().y
        'kp_bounds': (0.4, 3.0),
        'kv_bounds': (1.0, 3.0),
    },
    # 'z': {
    #     'idx': 1,           # sol.y index
    #     'cmd': 'z',         # Trajectory().z
    #     'kp_bounds': (0.8, 2.0),
    #     'kv_bounds': (1.5, 2.5),
    # },
    # 'phi':{'idx':2, 'cmd':'phi', ...},
}

def objective(trial):
    # 1) sample only the axes listed in `axes`
    sample = {}
    for ax, cfg in axes.items():
        sample[f"Kp_{ax}"] = trial.suggest_float(f"Kp_{ax}", *cfg['kp_bounds'])
        sample[f"Kv_{ax}"] = trial.suggest_float(f"Kv_{ax}", *cfg['kv_bounds'])

    # 2) build a new Gains dataclass overriding only those fields
    G = replace(Gains(), **sample)

    # 3) run sim
    quad = Quadrotor(Params(), G, Trajectory())
    sol  = Simulator(quad, x0=[0]*6, t_final=t_final).run()

    # 4) compute unit-less cost per axis and sum
    total_cost = 0.0
    for ax, cfg in axes.items():
        actual = sol.y[cfg['idx']]
        cmd    = np.array([getattr(quad.traj(t), cfg['cmd']) for t in sol.t])
        m      = StepResponseAnalyzer(sol.t, actual, cmd).compute_metrics()
        ov     = abs(m["overshoot_pct"] or 0) / 100.0          # → [0,∞)
        st     = (m["settling_time"] or t_final) / t_final     # → [0,1]
        total_cost += ((overshoot_weight*ov) + (settling_weight*st))

    return total_cost

if __name__ == "__main__":
    study = optuna.create_study(direction="minimize")
    study.optimize(objective, n_trials=200, timeout=600, n_jobs=1, show_progress_bar=False)

    print("Best gains:", study.best_params)
    print("Best cost :", study.best_value)
