"""
ME 684 - Chapter 2, Lab 1b: Are Our Equations Right?

01a_equations.py produced a number. Nobody checked it. This file asks the
same question of two answerers and compares:

    (A) PyBullet   the 'real robot'. Knows nothing about our algebra.
    (B) Our model  the equations we typed in 01a.

We compare ACCELERATION at a single instant, not a trajectory. Over a
trajectory the numerical integration adds errors of its own, and then a
mismatch would not tell us whether the equations or the integrator was at
fault. One instant leaves the equations nowhere to hide.

    python 01b_check.py

Next: 01c_trajectory.py runs both forward in time.
"""

import numpy as np                   # arrays, sin/cos, the matrix solve

# The constants come from our side, the robot from PyBullet's side.
from cartpole_model import G, M_CART, M_POLE, LC, I_POLE
from cartpole_robot import CartPole

M, m, lc, I = M_CART, M_POLE, LC, I_POLE

print("acceleration,  ours vs PyBullet")
print("   theta       F |    thddot sim      model     error")

# PyBullet updates velocity as v += a*dt, so (v_after - v_before)/dt hands
# back the exact acceleration it used -- this is not an approximation, and
# any dt works. Going much smaller actually hurts: a tiny difference divided
# by a tiny dt amplifies floating-point round-off.
dt = 1e-3                          # 1 ms

env = CartPole(gui=False, dt=dt)
rng = np.random.default_rng(0)     # fixed seed: everyone sees these numbers
worst = 0.0

for trial in range(6):

    # ============================================================
    #  THE QUESTION
    # ============================================================
    # Make up a random state and a random force. Big angles on
    # purpose: at theta = 0, sin(theta) = 0 would delete whole terms
    # from our equations and a mistake in them could pass unnoticed.

    s = rng.uniform(-0.4, 0.4, 4)      # [x, theta, xdot, thetadot]
    F = rng.uniform(-8, 8)             # force on the cart [N]
    th, thd = s[1], s[3]               # the two we need below

    # ============================================================
    #  ANSWER 1 -- PyBullet, the 'real robot'
    # ============================================================
    # Four calls, and each one matters:
    #
    #  env.reset(s)       Teleport. Writes theta and thetadot into
    #                     the joints directly. No physics runs here;
    #                     we are just placing the robot at the exact
    #                     state we want to ask about.
    #
    #  env.apply_force(F) Queue a force on the CART joint. PyBullet
    #                     clears applied forces after every step, so
    #                     it has to be re-applied each time -- and the
    #                     POLE joint gets nothing, because the pole
    #                     has no motor. That is the underactuation.
    #
    #  env.step()         Run the engine for exactly dt seconds.
    #
    #  env.get_state()    Read the joints back out.
    #
    # PyBullet never reports acceleration, only position and
    # velocity, so we recover it from how much the velocity changed.

    env.reset(s)
    env.apply_force(F)
    env.step()
    s_after = env.get_state()

    v_before = s[2:]                   # [xdot, thetadot] before
    v_after = s_after[2:]              # [xdot, thetadot] after
    sim = (v_after - v_before) / dt    # [xddot, thddot]

    # ============================================================
    #  ANSWER 2 -- our equations, from the board
    # ============================================================
    # The same matrix as 01a, now with F in the top row. Nothing here
    # knows PyBullet exists.

    mass = np.array([[M + m, m * lc * np.cos(th)],
                     [m * lc * np.cos(th), I + m * lc**2]])
    rhs = np.array([F + m * lc * np.sin(th) * thd**2,
                    m * G * lc * np.sin(th)])
    ours = np.linalg.solve(mass, rhs)  # [xddot, thddot]

    # ============================================================
    #  COMPARE
    # ============================================================
    err = np.abs(sim - ours).max()
    worst = max(worst, err)
    print(f"  {th:6.3f} {F:7.2f} | {sim[1]:12.5f} {ours[1]:10.5f} {err:9.1e}")

env.close()
print(f"  worst = {worst:.1e}  -- round-off. Our equations ARE the robot.")

# Try it: flip one sign in the mass matrix above and run again. The error
# jumps from 1e-14 to about 20. That is what this file is for.
