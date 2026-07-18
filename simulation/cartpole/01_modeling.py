"""
ME 684 - Chapter 2, Lab 1: Mathematical Modeling
=================================================

The question this lab answers:

    "We derived equations of motion on the board.
     Do they actually describe this robot?"

We compare three things:

    (A) PyBullet          the 'real robot'. It knows nothing about our algebra.
    (B) Nonlinear model   the Lagrange equations we derived, no approximations.
    (C) Linearized model  sdot = A s + B u, valid only near upright.

If (A) and (B) agree, our modeling is correct.
Watching where (B) and (C) diverge tells us how far we can trust linearization
-- which matters a great deal, because every controller in Chapter 3 is built
on (C).

HOW TO READ THIS FILE
    Top to bottom, once. There are no functions to jump to: the file runs in
    four parts, PART 0 through PART 3, in the order they are printed.

    state vector  s = [x, theta, xdot, thetadot]
                  x     = cart position   [m]
                  theta = pole angle from upright  [rad]
    input         F = horizontal force on the cart [N]

Run:
    python 01_modeling.py            # show plots interactively
    python 01_modeling.py --save     # only write PNGs into results/
"""

import sys
from pathlib import Path

import numpy as np
import matplotlib

SAVE_ONLY = "--save" in sys.argv
if SAVE_ONLY:
    matplotlib.use("Agg")           # no screen: draw straight to a file
import matplotlib.pyplot as plt

# CartPole is the PyBullet wrapper -- our stand-in for the real robot.
# rk4_step advances a model by one time step. Both are plumbing; you do not
# need to read them to follow this lab.
from cartpole_env import (
    CartPole, rk4_step, nonlinear_dynamics, linear_dynamics,
    G, M_CART, M_POLE, L_POLE, LC, I_POLE,
)

RESULTS = Path(__file__).parent / "results"
RESULTS.mkdir(exist_ok=True)
np.set_printoptions(precision=4, suppress=True)


# ======================================================================
#  PART 0.  The parameters, and the linear model we will control later
# ======================================================================
print("=" * 70)
print(" PART 0.  Physical parameters")
print("=" * 70)
print(f"  M  (cart mass)          = {M_CART} kg")
print(f"  m  (pole mass)          = {M_POLE} kg")
print(f"  L  (pole length)        = {L_POLE} m")
print(f"  lc (hinge -> COM)       = {LC} m")
print(f"  I  (inertia about COM)  = {I_POLE:.6f} kg m^2")
print(f"  g                       = {G} m/s^2")

# Build A and B right here, exactly as derived in Section 2.6 of the notes.
# Shorter names so the formulas below look like the ones on the board.
M, m, lc, I = M_CART, M_POLE, LC, I_POLE
D = (I + m * lc**2) * (M + m) - (m * lc) ** 2        # determinant of the 2x2 mass matrix

A = np.zeros((4, 4))
A[0, 2] = 1.0                                        # xdot     = xdot
A[1, 3] = 1.0                                        # thetadot = thetadot
A[2, 1] = -(m**2) * G * lc**2 / D                    # a1
A[3, 1] = (M + m) * m * G * lc / D                   # a2

B = np.zeros(4)
B[2] = (I + m * lc**2) / D                           # b1
B[3] = -m * lc / D                                   # b2

print("\n" + "=" * 70)
print(" Linearized model    sdot = A s + B u,   s = [x, theta, xdot, thetadot]")
print("=" * 70)
print("  A =\n", A)
print("\n  B =", B)

eigenvalues = np.linalg.eigvals(A)
unstable = eigenvalues[eigenvalues.real > 1e-9].real
print("\n  eigenvalues of A:", np.round(eigenvalues, 4))
print(f"\n  There is a pole at {unstable} , in the right half plane.")
print("  The upright equilibrium is UNSTABLE. Left alone, the pole always falls.")
print(f"\n  B[3] = {B[3]:.4f} < 0")
print("  Pushing the cart right (F > 0) rotates the pole LEFT.")
print("  So a pole falling to the right is caught by pushing RIGHT,")
print("  exactly like chasing a broomstick balanced on your palm.")


# ======================================================================
#  PART 1.  Do our equations give the same accelerations as the robot?
# ======================================================================
# Pick a random state, apply the same force to both, and compare the
# accelerations they produce. This is the sharpest possible test: it asks
# about one instant, so no integration error can creep in.
print("\n" + "=" * 70)
print(" PART 1.  Instantaneous acceleration:  our equations  vs  PyBullet")
print("=" * 70)

dt_tiny = 1e-4                      # tiny step: (v_after - v_before)/dt is then clean
env = CartPole(gui=False, dt=dt_tiny)
rng = np.random.default_rng(0)      # fixed seed, so everyone sees the same numbers

print(f"  {'theta [rad]':>12} {'F [N]':>8} | {'thddot (sim)':>13} "
      f"{'thddot (model)':>15} {'|error|':>10}")
print("  " + "-" * 66)

worst_error = 0.0
for trial in range(6):
    # Deliberately large angles and forces, so the nonlinear terms matter.
    s = rng.uniform(-0.4, 0.4, 4)
    F = rng.uniform(-8, 8)
    theta, thetadot = s[1], s[3]

    # --- the robot's answer: step once, then difference the velocities ---
    env.reset(s)
    env.apply_force(F)
    env.step()
    acc_sim = (env.get_state()[2:] - s[2:]) / dt_tiny          # [xddot, thetaddot]

    # --- our answer: the equations of motion, written out in full ---
    # [ M+m            m*lc*cos(th) ] [ xddot     ]   [ F + m*lc*sin(th)*thetadot^2 ]
    # [ m*lc*cos(th)   I + m*lc^2   ] [ thetaddot ] = [ m*g*lc*sin(th)              ]
    mass_matrix = np.array([[M + m,                m * lc * np.cos(theta)],
                            [m * lc * np.cos(theta), I + m * lc**2]])
    right_side = np.array([F + m * lc * np.sin(theta) * thetadot**2,
                           m * G * lc * np.sin(theta)])
    acc_model = np.linalg.solve(mass_matrix, right_side)       # [xddot, thetaddot]

    error = np.abs(acc_sim - acc_model).max()
    worst_error = max(worst_error, error)
    print(f"  {theta:>12.3f} {F:>8.2f} | {acc_sim[1]:>13.5f} "
          f"{acc_model[1]:>15.5f} {error:>10.2e}")

env.close()
print(f"\n  worst error = {worst_error:.2e}   (double-precision round-off)")
print("  Our equations of motion ARE the simulator's dynamics.")


# ======================================================================
#  PART 2.  Let it fall, and watch how long each model keeps up
# ======================================================================
# Release the pole from theta0 with no input at all, and advance all three
# in the SAME loop so they stay exactly in step with each other.
#
# Two settings worth understanding before you change them:
#   T  = 0.7 s   the unstable pole is at +3.97 rad/s (time constant 0.25 s),
#                so after ~2 s the pole has swung right over and comparing
#                angles stops meaning anything.
#   dt = 1/2000  finer than the usual 1/240. At 1/240 the integrator mismatch
#                (PART 3) is larger than the linearization error itself, and
#                the plot would suggest the linear model beats the nonlinear
#                one -- which is nonsense.
print("\n" + "=" * 70)
print(" PART 2.  Trajectories -- how far can we trust the linearization?")
print("=" * 70)

T = 0.7
dt = 1.0 / 2000.0
n_steps = int(T / dt)

for theta0_deg, tag in [(3.0, "small"), (30.0, "large")]:
    s0 = np.array([0.0, np.deg2rad(theta0_deg), 0.0, 0.0])

    env = CartPole(gui=False, dt=dt)
    env.reset(s0)
    s_nonlinear = s0.copy()
    s_linear = s0.copy()

    ts = np.zeros(n_steps)
    theta_sim = np.zeros(n_steps)
    theta_nonlinear = np.zeros(n_steps)
    theta_linear = np.zeros(n_steps)

    for k in range(n_steps):
        # record where all three are right now
        ts[k] = k * dt
        theta_sim[k] = env.get_state()[1]
        theta_nonlinear[k] = s_nonlinear[1]
        theta_linear[k] = s_linear[1]

        # advance all three by one step, with the same input F = 0
        F = 0.0
        env.apply_force(F)
        env.step()
        s_nonlinear = rk4_step(s_nonlinear, F, dt, nonlinear_dynamics)
        s_linear = rk4_step(s_linear, F, dt, linear_dynamics)

    env.close()

    err_nonlinear = np.rad2deg(np.abs(theta_sim - theta_nonlinear).max())
    err_linear = np.rad2deg(np.abs(theta_sim - theta_linear).max())
    print(f"\n  theta0 = {theta0_deg:>5.1f} deg, free fall for {T} s "
          f"(final theta = {np.rad2deg(theta_sim[-1]):.1f} deg)")
    print(f"    max |theta_sim - theta_nonlinear| = {err_nonlinear:9.4f} deg")
    print(f"    max |theta_sim - theta_linear|    = {err_linear:9.4f} deg")

    # --- top panel: the three angle histories on one axis ---
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ax[0].plot(ts, np.rad2deg(theta_sim), lw=3.5, alpha=0.35,
               label="(A) PyBullet  ['real robot']")
    ax[0].plot(ts, np.rad2deg(theta_nonlinear), "--", lw=1.6,
               label="(B) our nonlinear model")
    ax[0].plot(ts, np.rad2deg(theta_linear), ":", lw=1.8,
               label="(C) linearized model")
    ax[0].set_ylabel("theta [deg]")
    ax[0].legend(loc="upper left")
    ax[0].set_title(f"Free fall from theta0 = {theta0_deg} deg,  F = 0")

    # --- bottom panel: the two errors, on a log scale ---
    ax[1].plot(ts, np.rad2deg(np.abs(theta_sim - theta_nonlinear)), "--",
               label="|A - B|   model vs simulator")
    ax[1].plot(ts, np.rad2deg(np.abs(theta_sim - theta_linear)), ":",
               label="|A - C|   linearization error")
    ax[1].set_yscale("log")
    ax[1].set_ylabel("|error| [deg]")
    ax[1].set_xlabel("time [s]")
    ax[1].legend(loc="lower right")

    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()

    out = RESULTS / f"01_trajectory_{tag}.png"
    fig.savefig(out, dpi=120)
    print(f"    [saved] {out}")
    if not SAVE_ONLY:
        plt.show()
    plt.close(fig)


# ======================================================================
#  PART 3.  The leftover |A - B| gap: modeling error, or the integrator?
# ======================================================================
# PART 1 showed the two sets of equations agree to 1e-13, so the gap between
# trajectory (A) and trajectory (B) cannot be a modeling error. It is there
# because PyBullet integrates with semi-implicit Euler while we use RK4, and
# an unstable plant amplifies that difference exponentially.
#
# The test: shrink dt. Both integrators then converge to the same exact
# solution, so |A - B| must go to zero. |A - C| must NOT -- it converges to
# the genuine error we committed by replacing sin(th) with th and dropping
# the thetadot^2 term.
print("\n" + "=" * 70)
print(" PART 3.  Refine the time step:  which error vanishes, which does not?")
print("=" * 70)

theta0_deg = 3.0
s0 = np.array([0.0, np.deg2rad(theta0_deg), 0.0, 0.0])
print(f"  theta0 = {theta0_deg} deg, T = {T} s\n")
print(f"  {'dt [s]':>10} | {'max|A-B| [deg]':>15} {'max|A-C| [deg]':>15}")
print("  " + "-" * 45)

for dt_test in (1 / 240, 1 / 1000, 1 / 4000, 1 / 16000):
    # same three-in-one-loop pattern as PART 2, just with dt_test
    env = CartPole(gui=False, dt=dt_test)
    env.reset(s0)
    s_nonlinear = s0.copy()
    s_linear = s0.copy()

    gap_nonlinear = 0.0
    gap_linear = 0.0
    for k in range(int(T / dt_test)):
        theta_here = env.get_state()[1]
        gap_nonlinear = max(gap_nonlinear, abs(theta_here - s_nonlinear[1]))
        gap_linear = max(gap_linear, abs(theta_here - s_linear[1]))

        env.apply_force(0.0)
        env.step()
        s_nonlinear = rk4_step(s_nonlinear, 0.0, dt_test, nonlinear_dynamics)
        s_linear = rk4_step(s_linear, 0.0, dt_test, linear_dynamics)

    env.close()
    print(f"  {dt_test:>10.5f} | {np.rad2deg(gap_nonlinear):>15.6f} "
          f"{np.rad2deg(gap_linear):>15.6f}")

print("\n  |A-B| -> 0        : integration artifact, not a modeling error.")
print("  |A-C| -> constant : a real approximation error we chose to make.")


# ======================================================================
#  Takeaway
# ======================================================================
print("\n" + "=" * 70)
print(" Takeaway")
print("=" * 70)
print("  (B) is the simulator, up to integration error -> our modeling is right.")
print("  (C) is only the simulator near theta = 0.")
print("  Linear control theory, which we start in Chapter 3, buys its power by")
print("  giving up everything outside a neighborhood of the upright equilibrium.")
