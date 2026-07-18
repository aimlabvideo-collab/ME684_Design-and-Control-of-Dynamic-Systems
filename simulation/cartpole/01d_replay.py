"""
ME 684 - Chapter 2, Lab 1d: Watching Our Own Equations Move

Nothing is simulated in the viewer here. We compute the trajectory
ourselves, with our own equations, and then push each state into PyBullet
one frame at a time. The robot on screen is being driven by our algebra.

    python 01d_replay.py

A 3D window opens and plays the fall once.
"""

import time                          # time.sleep, to pace the replay
import numpy as np                   # arrays and math

from cartpole_env import CartPole, rk4_step, nonlinear_dynamics


# --- step 1: compute the trajectory, no simulator involved -------------

T = 0.7
dt = 1.0 / 2000.0
n = int(T / dt)

s = np.array([0.0, np.deg2rad(30.0), 0.0, 0.0])   # released from 30 deg
traj = np.zeros((n, 4))

for k in range(n):
    traj[k] = s
    s = rk4_step(s, 0.0, dt, nonlinear_dynamics)

print(f"computed {n} states with our own equations")
print(f"  final theta = {np.rad2deg(traj[-1, 1]):.1f} deg")


# --- step 2: play them back -------------------------------------------
# env.reset() writes a state straight into the joints, so calling it every
# frame turns the physics engine into nothing more than a renderer. We skip
# every 8th state to play at a watchable speed.

print("replaying in PyBullet")
env = CartPole(gui=True, dt=dt)

for k in range(0, n, 8):
    env.reset(traj[k])               # <-- our numbers, not PyBullet's
    time.sleep(8 * dt)

time.sleep(1.0)
env.close()

print("\nOur equations reproduce the robot. Chapter 3 builds controllers on")
print("a LINEARIZED version of them -- that is the next lab's job.")
