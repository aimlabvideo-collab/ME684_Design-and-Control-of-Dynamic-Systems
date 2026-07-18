"""
ME 684 - Chapter 2, Lab 1: Mathematical Modeling

    "We derived equations of motion on the board.
     Do they actually describe this robot?"

Three things get compared:

    (A) PyBullet          the 'real robot'. Knows nothing about our algebra.
    (B) Nonlinear model   the Lagrange equations we derived.
    (C) Linearized model  sdot = A s + B u, valid only near upright.

(A) vs (B) checks our modeling. (B) vs (C) shows how far linearization
can be trusted -- and every controller in Chapter 3 is built on (C).

Read top to bottom. No functions, four parts, in the order they print.

    state  s = [x, theta, xdot, thetadot]     x     cart position [m]
                                              theta angle from upright [rad]
    input  F = force on the cart [N]

    python 01_modeling.py           # show plots
    python 01_modeling.py --save    # write PNGs only
"""

import sys
from pathlib import Path

import numpy as np
import matplotlib

SAVE = "--save" in sys.argv
if SAVE:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Plumbing. You do not need to read cartpole_env.py to follow this lab.
from cartpole_env import (
    CartPole, rk4_step, nonlinear_dynamics, linear_dynamics,
    G, M_CART, M_POLE, LC, I_POLE,
)

RESULTS = Path(__file__).parent / "results"
RESULTS.mkdir(exist_ok=True)
np.set_printoptions(precision=4, suppress=True)

# Short names, so the formulas below look like the ones on the board.
M, m, lc, I = M_CART, M_POLE, LC, I_POLE


# --- PART 0: the linear model, built exactly as in Section 2.6 ---------

D = (I + m * lc**2) * (M + m) - (m * lc) ** 2   # det of the 2x2 mass matrix

A = np.zeros((4, 4))
A[0, 2] = 1.0                          # xdot     = xdot
A[1, 3] = 1.0                          # thetadot = thetadot
A[2, 1] = -(m**2) * G * lc**2 / D      # a1
A[3, 1] = (M + m) * m * G * lc / D     # a2

B = np.zeros(4)
B[2] = (I + m * lc**2) / D             # b1
B[3] = -m * lc / D                     # b2

print("PART 0.  sdot = A s + B u")
print("A =", A, sep="\n")
print("B =", B)
print("eig(A) =", np.round(np.linalg.eigvals(A), 3))

# One eigenvalue is positive: upright is UNSTABLE, the pole always falls.
# B[3] < 0: push the cart right and the pole rotates left. So you catch a
# pole falling right by pushing right -- like a broomstick on your palm.


# --- PART 1: same state, same force. Same acceleration? ---------------
# The sharpest test there is: it asks about one instant, so no integration
# error can creep in and hide a modeling mistake.

print("\nPART 1.  acceleration,  ours vs PyBullet")
print("   theta       F |    thddot sim      model     error")

dt = 1e-4                              # tiny, so (dv/dt) is a clean derivative
env = CartPole(gui=False, dt=dt)
rng = np.random.default_rng(0)         # fixed seed: everyone sees these numbers
worst = 0.0

for trial in range(6):
    s = rng.uniform(-0.4, 0.4, 4)      # big angles, to exercise the sin/cos
    F = rng.uniform(-8, 8)
    th, thd = s[1], s[3]

    env.reset(s)                       # the robot's answer
    env.apply_force(F)
    env.step()
    sim = (env.get_state()[2:] - s[2:]) / dt

    # our answer, straight off the board:
    #   [ M+m          m lc cos th ] [ xddot  ]   [ F + m lc sin th thd^2 ]
    #   [ m lc cos th  I + m lc^2  ] [ thddot ] = [ m g lc sin th         ]
    mass = np.array([[M + m, m * lc * np.cos(th)],
                     [m * lc * np.cos(th), I + m * lc**2]])
    rhs = np.array([F + m * lc * np.sin(th) * thd**2,
                    m * G * lc * np.sin(th)])
    ours = np.linalg.solve(mass, rhs)

    err = np.abs(sim - ours).max()
    worst = max(worst, err)
    print(f"  {th:6.3f} {F:7.2f} | {sim[1]:12.5f} {ours[1]:10.5f} {err:9.1e}")

env.close()
print(f"  worst = {worst:.1e}  -- round-off. Our equations ARE the robot.")


# --- PART 2: let it fall, see how long each model keeps up ------------
# All three advance in ONE loop, so they stay exactly in step.
#   T  = 0.7 s   unstable pole at +3.97 rad/s (0.25 s time constant), so
#                past ~2 s the pole has swung over and angles stop meaning
#                anything.
#   dt = 1/2000  finer than the usual 1/240. At 1/240 the integrator gap
#                (PART 3) exceeds the linearization error and the plot would
#                suggest (C) beats (B), which is nonsense.

print("\nPART 2.  free fall")
T, dt = 0.7, 1.0 / 2000.0
n = int(T / dt)

for th0_deg, tag in [(3.0, "small"), (30.0, "large")]:
    s0 = np.array([0.0, np.deg2rad(th0_deg), 0.0, 0.0])
    env = CartPole(gui=False, dt=dt)
    env.reset(s0)
    s_non = s0.copy()
    s_lin = s0.copy()

    ts = np.zeros(n)
    a = np.zeros(n)                    # (A) PyBullet
    b = np.zeros(n)                    # (B) nonlinear
    c = np.zeros(n)                    # (C) linear

    for k in range(n):
        ts[k] = k * dt                 # record all three right now
        a[k] = env.get_state()[1]
        b[k] = s_non[1]
        c[k] = s_lin[1]

        env.apply_force(0.0)           # then advance all three, F = 0
        env.step()
        s_non = rk4_step(s_non, 0.0, dt, nonlinear_dynamics)
        s_lin = rk4_step(s_lin, 0.0, dt, linear_dynamics)

    env.close()
    e_b = np.rad2deg(np.abs(a - b).max())
    e_c = np.rad2deg(np.abs(a - c).max())
    print(f"  theta0 = {th0_deg:4.1f} deg -> |A-B| = {e_b:7.4f},"
          f"  |A-C| = {e_c:8.4f} deg")

    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ax[0].plot(ts, np.rad2deg(a), lw=3.5, alpha=0.35, label="(A) PyBullet")
    ax[0].plot(ts, np.rad2deg(b), "--", lw=1.6, label="(B) nonlinear")
    ax[0].plot(ts, np.rad2deg(c), ":", lw=1.8, label="(C) linearized")
    ax[0].set_ylabel("theta [deg]")
    ax[0].set_title(f"Free fall from {th0_deg} deg,  F = 0")
    ax[0].legend(loc="upper left")

    ax[1].plot(ts, np.rad2deg(np.abs(a - b)), "--", label="|A-B| integrator")
    ax[1].plot(ts, np.rad2deg(np.abs(a - c)), ":", label="|A-C| linearization")
    ax[1].set_yscale("log")
    ax[1].set_ylabel("|error| [deg]")
    ax[1].set_xlabel("time [s]")
    ax[1].legend(loc="lower right")

    for x in ax:
        x.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(RESULTS / f"01_trajectory_{tag}.png", dpi=120)
    if not SAVE:
        plt.show()
    plt.close(fig)


# --- PART 3: is the leftover |A-B| gap our fault? ---------------------
# PART 1 showed both sets of equations agree to 1e-13, so the gap cannot be
# a modeling error. PyBullet integrates with semi-implicit Euler, we use
# RK4, and an unstable plant grows that difference exponentially.
# Shrink dt: both integrators converge to the same exact solution, so |A-B|
# must vanish. |A-C| must not -- that is the real price of dropping
# sin(th) -> th and the thdot^2 term.

print("\nPART 3.  shrink dt")
print("      dt |  |A-B| deg   |A-C| deg")
s0 = np.array([0.0, np.deg2rad(3.0), 0.0, 0.0])

for dt in (1 / 240, 1 / 1000, 1 / 4000, 1 / 16000):
    env = CartPole(gui=False, dt=dt)
    env.reset(s0)
    s_non = s0.copy()
    s_lin = s0.copy()
    gap_b = gap_c = 0.0

    for k in range(int(T / dt)):
        th = env.get_state()[1]
        gap_b = max(gap_b, abs(th - s_non[1]))
        gap_c = max(gap_c, abs(th - s_lin[1]))

        env.apply_force(0.0)
        env.step()
        s_non = rk4_step(s_non, 0.0, dt, nonlinear_dynamics)
        s_lin = rk4_step(s_lin, 0.0, dt, linear_dynamics)

    env.close()
    print(f"  {dt:6.5f} | {np.rad2deg(gap_b):9.5f} {np.rad2deg(gap_c):11.5f}")

print("  |A-B| -> 0        integration artifact, not a modeling error")
print("  |A-C| -> constant a real approximation, and we chose to make it")
print("\n(C) is the robot only near theta = 0. That neighborhood is what")
print("linear control theory trades away its generality for.")
