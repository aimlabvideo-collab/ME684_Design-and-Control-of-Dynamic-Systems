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

Run:
    python 01_modeling.py            # show plots interactively
    python 01_modeling.py --save     # only write PNGs into results/
"""

import sys
from pathlib import Path

import numpy as np

from cartpole_env import (
    CartPole, nonlinear_dynamics, linear_dynamics, linearized_model,
    rollout_model, G, M_CART, M_POLE, L_POLE, LC, I_POLE,
)

SAVE_ONLY = "--save" in sys.argv
RESULTS = Path(__file__).parent / "results"


# ----------------------------------------------------------------------
# Step 0. Print the parameters and the linearized model
# ----------------------------------------------------------------------
def print_model():
    print("=" * 70)
    print(" Step 0.  Physical parameters")
    print("=" * 70)
    print(f"  M  (cart mass)          = {M_CART} kg")
    print(f"  m  (pole mass)          = {M_POLE} kg")
    print(f"  L  (pole length)        = {L_POLE} m")
    print(f"  lc (hinge -> COM)       = {LC} m")
    print(f"  I  (inertia about COM)  = {I_POLE:.6f} kg m^2")
    print(f"  g                       = {G} m/s^2")

    A, B = linearized_model()
    np.set_printoptions(precision=4, suppress=True)

    print("\n" + "=" * 70)
    print(" Linearized model    sdot = A s + B u,   s = [x, theta, xdot, thetadot]")
    print("=" * 70)
    print("  A =\n", A)
    print("\n  B^T =", B.T)

    eig = np.linalg.eigvals(A)
    print("\n  eigenvalues of A:", np.round(eig, 4))
    unstable = eig[eig.real > 1e-9].real
    print(f"\n  There is a pole at {unstable} , in the right half plane.")
    print("  The upright equilibrium is UNSTABLE. Left alone, the pole always falls.")
    print(f"\n  B[3] = {B[3, 0]:.4f} < 0")
    print("  Pushing the cart right (F > 0) rotates the pole LEFT.")
    print("  So a pole falling to the right is caught by pushing RIGHT,")
    print("  exactly like chasing a broomstick balanced on your palm.")
    return A, B


# ----------------------------------------------------------------------
# Step 1. Instantaneous accelerations: our equations vs the simulator
# ----------------------------------------------------------------------
def compare_accelerations():
    """Pick random states, apply the same force to both, compare accelerations.

    We get the simulator's acceleration by stepping once and taking
    (v_after - v_before) / dt, so we use a very small dt to keep the finite
    difference clean.
    """
    print("\n" + "=" * 70)
    print(" Step 1.  Instantaneous acceleration:  our equations  vs  PyBullet")
    print("=" * 70)

    dt = 1e-4
    env = CartPole(gui=False, dt=dt)
    rng = np.random.default_rng(0)

    print(f"  {'theta [rad]':>12} {'F [N]':>8} | {'thddot (sim)':>13} "
          f"{'thddot (model)':>15} {'|error|':>10}")
    print("  " + "-" * 66)

    worst = 0.0
    for _ in range(6):
        # Deliberately large angles, so the nonlinear terms are exercised too.
        s = rng.uniform(-0.4, 0.4, 4)
        F = rng.uniform(-8, 8)

        env.reset(s)
        env.apply_force(F)
        env.step()
        acc_sim = (env.get_state()[2:] - s[2:]) / dt      # [xddot, thddot]
        acc_model = nonlinear_dynamics(s, F)[2:]

        err = np.abs(acc_sim - acc_model).max()
        worst = max(worst, err)
        print(f"  {s[1]:>12.3f} {F:>8.2f} | {acc_sim[1]:>13.5f} "
              f"{acc_model[1]:>15.5f} {err:>10.2e}")

    env.close()
    print(f"\n  worst error = {worst:.2e}   (double-precision round-off)")
    print("  Our equations of motion ARE the simulator's dynamics.")


# ----------------------------------------------------------------------
# Step 2. Trajectories: let it fall, and see how long each model tracks
# ----------------------------------------------------------------------
def simulate_pybullet(s0, U, dt):
    """Roll the simulator forward through the input sequence U."""
    env = CartPole(gui=False, dt=dt)
    env.reset(s0)
    S = np.zeros((len(U), 4))
    for k in range(len(U)):
        S[k] = env.get_state()
        env.apply_force(U[k])
        env.step()
    env.close()
    return S


def free_fall(theta0_deg, T, dt):
    """Release from theta0 with zero input. Returns (ts, S_sim, S_nonlinear, S_linear).

    T is kept short on purpose. The unstable pole is at +3.97 rad/s, i.e. a time
    constant of 0.25 s, so after ~2 s the pole has swung all the way over and any
    comparison of angles becomes meaningless.
    """
    n = int(T / dt)
    s0 = np.array([0.0, np.deg2rad(theta0_deg), 0.0, 0.0])
    U = np.zeros(n)                       # no input at all -- just let it fall

    S_sim = simulate_pybullet(s0, U, dt)
    S_non = rollout_model(nonlinear_dynamics, s0, U, dt)
    S_lin = rollout_model(linear_dynamics, s0, U, dt)
    return np.arange(n) * dt, S_sim, S_non, S_lin


def compare_trajectories(theta0_deg, T=0.7, tag="", dt=1.0 / 2000.0):
    """Plot (A), (B), (C) against each other.

    dt is finer than the usual 1/240 here. At 1/240 the integrator mismatch
    (see step_size_study) is larger than the linearization error itself, and the
    plot would misleadingly suggest the linear model beats the nonlinear one.
    """
    ts, S_sim, S_non, S_lin = free_fall(theta0_deg, T, dt)

    err_non = np.rad2deg(np.abs(S_sim[:, 1] - S_non[:, 1]).max())
    err_lin = np.rad2deg(np.abs(S_sim[:, 1] - S_lin[:, 1]).max())

    print(f"\n  theta0 = {theta0_deg:>5.1f} deg, free fall for {T} s "
          f"(final theta = {np.rad2deg(S_sim[-1, 1]):.1f} deg)")
    print(f"    max |theta_sim - theta_nonlinear| = {err_non:9.4f} deg")
    print(f"    max |theta_sim - theta_linear|    = {err_lin:9.4f} deg")

    import matplotlib
    if SAVE_ONLY:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ax[0].plot(ts, np.rad2deg(S_sim[:, 1]), lw=3.5, alpha=0.35,
               label="(A) PyBullet  ['real robot']")
    ax[0].plot(ts, np.rad2deg(S_non[:, 1]), "--", lw=1.6,
               label="(B) our nonlinear model")
    ax[0].plot(ts, np.rad2deg(S_lin[:, 1]), ":", lw=1.8,
               label="(C) linearized model")
    ax[0].set_ylabel("theta [deg]")
    ax[0].legend(loc="upper left")
    ax[0].set_title(f"Free fall from theta0 = {theta0_deg} deg,  F = 0")

    ax[1].plot(ts, np.rad2deg(np.abs(S_sim[:, 1] - S_non[:, 1])), "--",
               label="|A - B|   model vs simulator")
    ax[1].plot(ts, np.rad2deg(np.abs(S_sim[:, 1] - S_lin[:, 1])), ":",
               label="|A - C|   linearization error")
    ax[1].set_yscale("log")
    ax[1].set_ylabel("|error| [deg]")
    ax[1].set_xlabel("time [s]")
    ax[1].legend(loc="lower right")

    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()

    RESULTS.mkdir(exist_ok=True)
    out = RESULTS / f"01_trajectory_{tag}.png"
    fig.savefig(out, dpi=120)
    print(f"    [saved] {out}")
    if not SAVE_ONLY:
        plt.show()
    plt.close(fig)


# ----------------------------------------------------------------------
# Step 3. Is the leftover |A - B| gap a modeling error? No -- it is the integrator.
# ----------------------------------------------------------------------
def step_size_study(theta0_deg=3.0, T=0.7):
    """Shrink dt and watch the two errors behave completely differently.

    Step 1 already showed the two sets of equations produce identical
    accelerations to 1e-13. So any gap between trajectory (A) and trajectory (B)
    cannot be a modeling error -- it is because PyBullet integrates with
    semi-implicit Euler while we integrate with RK4, and an unstable plant
    amplifies that difference exponentially.

    The test: as dt -> 0, both integrators converge to the same exact solution,
    so |A - B| must go to zero. |A - C| must NOT: it converges to the genuine
    error committed by dropping sin(th) - th and the thdot^2 term.
    """
    print("\n" + "=" * 70)
    print(" Step 3.  Refine the time step:  which error vanishes, which does not?")
    print("=" * 70)
    print(f"  theta0 = {theta0_deg} deg, T = {T} s\n")
    print(f"  {'dt [s]':>10} | {'max|A-B| [deg]':>15} {'max|A-C| [deg]':>15}")
    print("  " + "-" * 45)

    for dt in (1 / 240, 1 / 1000, 1 / 4000, 1 / 16000):
        _, S_sim, S_non, S_lin = free_fall(theta0_deg, T, dt)
        e_non = np.rad2deg(np.abs(S_sim[:, 1] - S_non[:, 1]).max())
        e_lin = np.rad2deg(np.abs(S_sim[:, 1] - S_lin[:, 1]).max())
        print(f"  {dt:>10.5f} | {e_non:>15.6f} {e_lin:>15.6f}")

    print("\n  |A-B| -> 0        : integration artifact, not a modeling error.")
    print("  |A-C| -> constant : a real approximation error we chose to make.")


if __name__ == "__main__":
    print_model()
    compare_accelerations()

    print("\n" + "=" * 70)
    print(" Step 2.  Trajectories -- how far can we trust the linearization?")
    print("=" * 70)
    compare_trajectories(theta0_deg=3.0, tag="small")     # (C) still tracks
    compare_trajectories(theta0_deg=30.0, tag="large")    # (C) falls apart

    step_size_study()

    print("\n" + "=" * 70)
    print(" Takeaway")
    print("=" * 70)
    print("  (B) is the simulator, up to integration error -> our modeling is right.")
    print("  (C) is only the simulator near theta = 0.")
    print("  Linear control theory, which we start in Chapter 3, buys its power by")
    print("  giving up everything outside a neighborhood of the upright equilibrium.")
