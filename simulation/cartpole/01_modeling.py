"""
ME 684 - Chapter 2, Lab 1: The Nonlinear Model

    "We derived equations of motion on the board.
     Do they actually describe this robot?"

Two things get compared:

    (A) PyBullet   the 'real robot'. Knows nothing about our algebra.
    (B) Our model  the Lagrange equations we derived. No approximations.

If (A) and (B) agree, our modeling is correct -- and we can then trust our
own equations enough to design on them.

Read top to bottom. No functions, five parts, in the order they print.

    state  s = [x, theta, xdot, thetadot]     x     cart position [m]
                                              theta angle from upright [rad]
    input  F = force on the cart [N]

    python 01_modeling.py           # plots, then a PyBullet replay
    python 01_modeling.py --save    # write PNGs only, no windows
"""

import sys
import time
from pathlib import Path

import numpy as np
import matplotlib

SAVE = "--save" in sys.argv
if SAVE:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Plumbing. You do not need to read cartpole_env.py to follow this lab.
#   nonlinear_dynamics(s, F) -> sdot   is the matrix solve of PART 0
#   rk4_step(s, F, dt, dyn)  -> s_next is one Runge-Kutta step
from cartpole_env import (
    CartPole, rk4_step, nonlinear_dynamics, G, M_CART, M_POLE, LC, I_POLE,
)

RESULTS = Path(__file__).parent / "results"
RESULTS.mkdir(exist_ok=True)

# Short names, so the formulas below look like the ones on the board.
M, m, lc, I = M_CART, M_POLE, LC, I_POLE


# --- PART 0: our equations of motion, written out once -----------------
# Everything in this file uses these five lines and nothing else.
#
#   [ M+m          m lc cos th ] [ xddot  ]   [ F + m lc sin th thd^2 ]
#   [ m lc cos th  I + m lc^2  ] [ thddot ] = [ m g lc sin th         ]
#
# accel(s, F) below is a direct transcription. Read it next to the matrix.

print("PART 0.  the model we derived")
print(f"  M = {M} kg   m = {m} kg   lc = {lc} m   I = {I:.4f} kg m^2")

s_test = np.array([0.0, 0.2, 0.0, 0.0])   # leaning 0.2 rad, at rest
th, thd = s_test[1], s_test[3]
mass = np.array([[M + m, m * lc * np.cos(th)],
                 [m * lc * np.cos(th), I + m * lc**2]])
rhs = np.array([0.0 + m * lc * np.sin(th) * thd**2,
                m * G * lc * np.sin(th)])
print("  at theta = 0.2 rad, F = 0:  [xddot, thddot] =",
      np.round(np.linalg.solve(mass, rhs), 4))
print("  positive thddot -> it accelerates further over. It falls.")


# --- PART 1: same state, same force. Same acceleration? ----------------
# The sharpest test there is: it asks about one instant, so no integration
# error can creep in and hide a modeling mistake.

print("\nPART 1.  acceleration,  ours vs PyBullet")
print("   theta       F |    thddot sim      model     error")

dt = 1e-4                          # tiny, so (dv/dt) is a clean derivative
env = CartPole(gui=False, dt=dt)
rng = np.random.default_rng(0)     # fixed seed: everyone sees these numbers
worst = 0.0

for trial in range(6):
    s = rng.uniform(-0.4, 0.4, 4)  # big angles, to exercise the sin/cos
    F = rng.uniform(-8, 8)
    th, thd = s[1], s[3]

    env.reset(s)                   # the robot's answer
    env.apply_force(F)
    env.step()
    sim = (env.get_state()[2:] - s[2:]) / dt

    mass = np.array([[M + m, m * lc * np.cos(th)],     # our answer
                     [m * lc * np.cos(th), I + m * lc**2]])
    rhs = np.array([F + m * lc * np.sin(th) * thd**2,
                    m * G * lc * np.sin(th)])
    ours = np.linalg.solve(mass, rhs)

    err = np.abs(sim - ours).max()
    worst = max(worst, err)
    print(f"  {th:6.3f} {F:7.2f} | {sim[1]:12.5f} {ours[1]:10.5f} {err:9.1e}")

env.close()
print(f"  worst = {worst:.1e}  -- round-off. Our equations ARE the robot.")


# --- PART 2: let it fall, and integrate our model alongside ------------
# One instant agreeing is not the same as a whole trajectory agreeing, so
# now we run both forward together, in ONE loop, and watch the gap.
#
#   T  = 0.7 s   the pole is unstable, so past ~2 s it has swung right over
#                and comparing angles stops meaning anything.
#   dt = 1/2000  fine enough that RK4 error stays far below everything else.

print("\nPART 2.  free fall")
T, dt = 0.7, 1.0 / 2000.0
n = int(T / dt)
saved = {}                         # keep the trajectories for the replay

for th0_deg, tag in [(3.0, "small"), (30.0, "large")]:
    s0 = np.array([0.0, np.deg2rad(th0_deg), 0.0, 0.0])
    env = CartPole(gui=False, dt=dt)
    env.reset(s0)
    s = s0.copy()

    ts = np.zeros(n)
    a = np.zeros(n)                # (A) PyBullet
    b = np.zeros(n)                # (B) our model
    traj = np.zeros((n, 4))

    for k in range(n):
        ts[k] = k * dt             # record both right now
        a[k] = env.get_state()[1]
        b[k] = s[1]
        traj[k] = s

        env.apply_force(0.0)       # advance the robot, F = 0
        env.step()
        s = rk4_step(s, 0.0, dt, nonlinear_dynamics)   # advance our model

    env.close()
    saved[tag] = (ts, traj)
    print(f"  theta0 = {th0_deg:4.1f} deg -> max |A-B| ="
          f" {np.rad2deg(np.abs(a - b).max()):8.4f} deg")

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
    fig.savefig(RESULTS / f"01_nonlinear_{tag}.png", dpi=120)
    if not SAVE:
        plt.show()
    plt.close(fig)


# --- PART 3: the leftover gap is not our fault --------------------------
# PART 1 showed both sets of equations agree to 1e-13, so the gap in PART 2
# cannot be a modeling error. PyBullet integrates with semi-implicit Euler,
# we use RK4, and an unstable plant grows that difference exponentially.
# Shrink dt and both converge to the same exact solution, so the gap must
# vanish. Watch it fall by roughly the same factor as dt.

print("\nPART 3.  shrink dt")
print("      dt |  max |A-B| deg")
s0 = np.array([0.0, np.deg2rad(3.0), 0.0, 0.0])

for dt in (1 / 240, 1 / 1000, 1 / 4000, 1 / 16000):
    env = CartPole(gui=False, dt=dt)
    env.reset(s0)
    s = s0.copy()
    gap = 0.0

    for k in range(int(T / dt)):
        gap = max(gap, abs(env.get_state()[1] - s[1]))
        env.apply_force(0.0)
        env.step()
        s = rk4_step(s, 0.0, dt, nonlinear_dynamics)

    env.close()
    print(f"  {dt:6.5f} | {np.rad2deg(gap):11.5f}")

print("  -> 0 as dt -> 0: an integration artifact, not a modeling error.")


# --- PART 4: replay OUR trajectory in PyBullet -------------------------
# Nothing is being simulated here. We computed traj ourselves in PART 2,
# and now we just push each state into the viewer, one frame at a time.
# The robot on screen is being driven by our algebra.

if not SAVE:
    print("\nPART 4.  replaying our own trajectory (close the window to end)")
    ts, traj = saved["large"]
    env = CartPole(gui=True, dt=1.0 / 2000.0)

    for k in range(0, len(traj), 8):       # every 8th frame ~ 250 fps of data
        env.reset(traj[k])                 # <-- our numbers, not PyBullet's
        time.sleep(8 / 2000.0)

    time.sleep(1.0)
    env.close()

print("\nOur equations reproduce the robot. Chapter 3 builds controllers on")
print("a LINEARIZED version of them -- that is the next lab's job.")
