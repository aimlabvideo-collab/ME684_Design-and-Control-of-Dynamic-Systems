"""
ME 684 - Chapter 2, Lab 1a: Writing Our Equations in Python

We derived the cart-pole equations of motion on the board. Step one is to
type them in and solve them at a single instant -- nothing more.

    state  s = [x, theta, xdot, thetadot]     x     cart position [m]
                                              theta angle from upright [rad]
    input  F = force on the cart [N]

    python 01a_equations.py

Next: 01b_check.py asks whether these equations are actually right.
"""

import numpy as np                   # arrays, sin/cos, the matrix solve

# Physical constants only. No simulator involved in this file.
from cartpole_env import G, M_CART, M_POLE, LC, I_POLE

# Short names, so the formulas below look like the ones on the board.
# Parameters are in SI units: kg, m, s.
M, m, lc, I = M_CART, M_POLE, LC, I_POLE

print("the model we derived")
print(f"  M = {M} kg   m = {m} kg   lc = {lc} m   I = {I:.4f} kg m^2")


# --- the equations of motion -------------------------------------------
#
#   [ M+m          m lc cos th ] [ xddot  ]   [ F + m lc sin th thd^2 ]
#   [ m lc cos th  I + m lc^2  ] [ thddot ] = [ m g lc sin th         ]
#
# Two equations, two unknowns: xddot and thddot. Below is a direct
# transcription -- read it next to the matrix above.

s_test = np.array([0.0, 0.2, 0.0, 0.0])   # leaning 0.2 rad, at rest
th, thd = s_test[1], s_test[3]
F = 0.0                                   # no push, just let gravity act

mass = np.array([[M + m, m * lc * np.cos(th)],          # left-hand matrix
                 [m * lc * np.cos(th), I + m * lc**2]])
rhs = np.array([F + m * lc * np.sin(th) * thd**2,       # right-hand vector
                m * G * lc * np.sin(th)])


# --- solve at this one instant -----------------------------------------
# np.linalg.solve solves the linear system  mass * unknowns = rhs.
# It is linear algebra, not a differential equation solver: no time
# passes, and the answer is the acceleration RIGHT NOW, at this state.

accel = np.linalg.solve(mass, rhs)
xddot = accel[0]
thddot = accel[1]

print("  at theta = 0.2 rad, F = 0:")
print(f"    xddot  = {xddot:.4f} m/s^2")
print(f"    thddot = {thddot:.4f} rad/s^2")
print("  thddot is positive -> it accelerates further over. It falls.")

# A number is not a check. These equations would print something just as
# plausible with a sign wrong somewhere. That is what 01b_check.py is for.
