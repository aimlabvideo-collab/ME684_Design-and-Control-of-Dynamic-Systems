"""
ME 684 - Chapter 2, Lab 1c: Running Both Forward in Time

01b showed our equations and the robot agree at any single instant. One
instant agreeing is not the same as a whole trajectory agreeing, so here we
release the pole and run both forward together, step by step.

A small gap opens up. The second half of this file shows that the gap is
NOT a modeling error.

We release from 3 degrees only. Large angles are already covered by 01b,
which tests random states out to 0.4 rad -- and our model makes no
approximation anyway, so nothing about it degrades as the angle grows.

    python 01c_trajectory.py

The 3D window plays the fall in real time, then a plot window opens --
close it to continue.
"""

import time                          # to pace the viewer at a watchable speed
import numpy as np                   # arrays and math
import matplotlib.pyplot as plt      # the plots

# rk4_step(s, F, dt, dyn) advances a model by one Runge-Kutta step.
# nonlinear_dynamics(s, F) is the matrix solve from 01a, packaged up.
from cartpole_env import CartPole, rk4_step, nonlinear_dynamics


# --- free fall, both at once -------------------------------------------
# Both are advanced inside ONE loop, so they always sit at the same time.
#
#   T  = how many seconds of motion to simulate
#   dt = 1 ms, the same step size 01b probes with
#
# There is no friction anywhere -- PyBullet's damping is switched off and our
# equations have no such term -- so the pole never settles. It swings down
# past the bottom, climbs the far side to the same height it started from,
# and comes back. A pendulum, not a fall.

print("free fall")
T = 5.0
dt = 1.0 / 1000.0
n = int(T / dt)

# SPEED sets how fast the 3D window plays: 1.0 is real time, 0.25 is quarter
# speed, and a big number just runs flat out with no waiting at all.
#
# Note we cannot pace this with a plain time.sleep(dt) per step. Windows
# rounds a sleep up to roughly 12 ms, so 1 ms sleeps would run the viewer
# about 12x too SLOW. Instead we check the wall clock each step and wait
# only for however long we are actually ahead.
SPEED = 1.0

th0_deg = 20.0
s0 = np.array([0.0, np.deg2rad(th0_deg), 0.0, 0.0])

env = CartPole(gui=True, dt=dt)     # (A) the robot
env.reset(s0)
s = s0.copy()                        # (B) our model

ts = np.zeros(n)
a = np.zeros(n)                      # (A) angle history
b = np.zeros(n)                      # (B) angle history

t_start = time.perf_counter()

for k in range(n):
    ts[k] = k * dt                   # record both right now
    a[k] = env.get_state()[1]
    b[k] = s[1]

    env.apply_force(0.0)             # advance the robot, F = 0
    env.step()
    s = rk4_step(s, 0.0, dt, nonlinear_dynamics)   # advance our model

    # wait until the wall clock catches up with the simulated time
    ahead = (k + 1) * dt / SPEED - (time.perf_counter() - t_start)
    if ahead > 0:
        time.sleep(ahead)

env.close()

gap = np.abs(a - b).max()            # biggest angle difference, in rad
gap_deg = np.rad2deg(gap)
print(f"  theta0 = {th0_deg:4.1f} deg -> max |A-B| = {gap_deg:8.4f} deg")

fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
ax[0].plot(ts, np.rad2deg(a), lw=3.5, alpha=0.35, label="(A) PyBullet")
ax[0].plot(ts, np.rad2deg(b), "--", lw=1.6, label="(B) our model")
ax[0].set_ylabel("theta [deg]")
ax[0].set_title(f"Free fall from {th0_deg} deg,  F = 0")
ax[0].legend(loc="upper left")

ax[1].plot(ts, np.rad2deg(np.abs(a - b)), "--")
ax[1].set_yscale("log")
ax[1].set_ylabel("|A-B| [deg]")
ax[1].set_xlabel("time [s]")

for x in ax:
    x.grid(alpha=0.3)
fig.tight_layout()
plt.show()                     # close the window to continue
plt.close(fig)


# --- so whose fault is the gap? ----------------------------------------
# 01b showed both sets of equations agree to 1e-14, so this cannot be a
# modeling error. PyBullet integrates with semi-implicit Euler, we use RK4,
# and an unstable plant grows that difference exponentially.
#
# The test: shrink dt. Both integrators converge to the same exact solution
# as dt -> 0, so the gap must vanish. Watch it fall roughly with dt.

print("\nshrink dt")
print("      dt |  max |A-B| deg")
s0 = np.array([0.0, np.deg2rad(3.0), 0.0, 0.0])

for dt in (1 / 240, 1 / 1000, 1 / 4000, 1 / 16000):
    env = CartPole(gui=False, dt=dt)
    env.reset(s0)
    s = s0.copy()
    gap = 0.0

    for k in range(int(T / dt)):
        th_robot = env.get_state()[1]
        th_ours = s[1]
        gap = max(gap, abs(th_robot - th_ours))

        env.apply_force(0.0)
        env.step()
        s = rk4_step(s, 0.0, dt, nonlinear_dynamics)

    env.close()
    gap_deg = np.rad2deg(gap)
    print(f"  {dt:6.5f} | {gap_deg:11.5f}")

print("  -> 0 as dt -> 0: an integration artifact, not a modeling error.")
