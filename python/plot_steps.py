"""Plot a step stream against the planned trajectory.

Usage:
    build-host/dump_steps retarget > steps.csv
    python python/plot_steps.py steps.csv

The CSV comes from test/host/dump_steps.cpp:
    ideal,t,p,v,a   planned trajectory (1 ms samples)
    step,t,pos,dir  emitted steps

Top: planned position vs the step count. Middle: planned velocity vs the step rate measured
from consecutive step times (what the motor actually gets). Bottom: tracking error at each step.
"""
import csv
import sys

import matplotlib.pyplot as plt
import numpy as np


def load(path):
    ideal, steps = [], []
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if not row:
                continue
            if row[0] == "ideal":
                ideal.append([float(x) for x in row[1:5]])
            elif row[0] == "step":
                steps.append([float(row[1]), float(row[2]), float(row[3])])
    return np.array(ideal), np.array(steps)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    ideal, steps = load(sys.argv[1])
    t_i, p_i, v_i, a_i = ideal.T
    t_s, pos_s, dir_s = steps.T

    # Step rate between consecutive steps, signed by direction, at the interval midpoint.
    dt = np.diff(t_s)
    rate = dir_s[1:] / np.where(dt > 0, dt, np.nan)
    t_rate = 0.5 * (t_s[1:] + t_s[:-1])

    err = np.interp(t_s, t_i, p_i) - pos_s

    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(10, 9))
    ax[0].plot(t_i, p_i, label="planned p")
    ax[0].step(t_s, pos_s, where="post", label="steps", alpha=0.7)
    ax[0].set_ylabel("position [steps]")
    ax[0].legend()
    ax[1].plot(t_i, v_i, label="planned v")
    ax[1].plot(t_rate, rate, ".", markersize=2, label="step rate")
    ax[1].set_ylabel("velocity [steps/s]")
    ax[1].legend()
    ax[2].plot(t_i, a_i)
    ax[2].set_ylabel("acceleration [steps/s²]")
    ax[3].plot(t_s, err, ".", markersize=2)
    ax[3].axhline(0.5, color="gray", lw=0.5)
    ax[3].axhline(-0.5, color="gray", lw=0.5)
    ax[3].set_ylabel("p(t_k) - pos_k [steps]")
    ax[3].set_xlabel("t [s]")
    fig.suptitle(f"{sys.argv[1]}: {len(t_s)} steps, max |error| = {np.max(np.abs(err)):.3f} steps")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
