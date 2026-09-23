"""
ME 684 - Chapter 2: the equations we derived, solved and then integrated

We worked out the cart-pole equations of motion on the board. This file
does two things with them, in order:

    1. solve them at ONE instant   -> the acceleration right now
    2. integrate them through time -> the whole motion, by stepping

    python model_equations.py            solve, integrate, plot
    python model_equations.py --save     headless, PNG to results/

    state  s = [x, theta, xdot, thetadot]
    x      cart position [m], +x to the right
    theta  pole angle from straight up [rad], +theta leans right
    F      force on the cart [N], +F pushes right

No simulator here. Just our algebra and numpy. Read it top to bottom.

Next: model_test.py asks whether these equations are actually right.
"""

import sys
import numpy as np
import matplotlib.pyplot as plt

# --- physical constants -------------------------------------------------
# These match assets/cartpole.urdf.

G = 9.81                          # gravity              [m/s^2]
M = 1.0                           # cart mass       M    [kg]
m = 0.1                           # pole mass       m    [kg]
L = 1.0                           # pole length     L    [m]
W = 0.05                          # pole width           [m]
lc = L / 2                        # hinge -> pole centre [m]
I = m * (L**2 + W**2) / 12        # pole inertia about its centre [kg m^2]

SAVE = "--save" in sys.argv

print("the model we derived")
print(f"  M = {M} kg   m = {m} kg   lc = {lc} m   I = {I:.5f} kg m^2")


# =======================================================================
#  PART 1 -- the equations at one instant
# =======================================================================
#
#   [ M+m           m lc cos th ] [ xddot  ]   [ F + m lc sin th thd^2 ]
#   [ m lc cos th   I + m lc^2  ] [ thddot ] = [ m g lc sin th         ]
#
# Two equations, two unknowns: xddot and thddot. Below is a direct
# transcription -- read it next to the matrix above.

# the model at one instant - typed from the matrix
s = np.array([0.0, 0.2, 0.0, 0.0])    # lean 0.2 rad, at rest
th, thd = s[1], s[3]
F = 0.0                               # no push, just let gravity act

# left side: the mass matrix
mass = np.array([[M + m,               m * lc * np.cos(th)],
                 [m * lc * np.cos(th), I + m * lc**2]])

# right side: the forces
rhs = np.array([F + m * lc * np.sin(th) * thd**2,
                m * G * lc * np.sin(th)])

# let the computer do the algebra.
# np.linalg.solve solves  mass @ accel = rhs.  It is linear algebra, not a
# differential-equation solver: no time passes. The answer is the
# acceleration RIGHT NOW, at this state.
accel = np.linalg.solve(mass, rhs)
xddot, thddot = accel

print(f"\nPart 1 -- one instant, at theta = {s[1]} rad, F = {F}:")
print(f"    xddot  = {xddot:+.4f} m/s^2")
print(f"    thddot = {thddot:+.4f} rad/s^2")
print("  thddot is positive -> it accelerates further over. It falls.")


# =======================================================================
#  PART 2 -- integrate through time
# =======================================================================
#
# Part 1 gave us the acceleration at one state. Acceleration is the rate of
# change of velocity, and velocity is the rate of change of position. So
# over a short step dt:
#
#     velocity_new = velocity + acceleration * dt
#     position_new = position + velocity_new * dt
#
# Do that over and over and the state walks forward in time. That is
# numerical integration (this simple version is called Euler's method).
# The smaller dt, the closer it tracks the true continuous motion.
#
# Inside the loop, the SAME matrix as Part 1 is solved again at every step,
# because the accelerations change as theta changes.

dt = 1.0 / 1000          # step size [s]
T = 1                  # how long to run [s]

s = np.array([0.0, 0.2, 0.0, 0.0])    # start over from the same lean, at rest
F = 0.0

ts = [0.0]
xs = [s[0]]
thetas = [s[1]]

for step in range(int(T / dt)):
    # s = [p, th, pdot, thdot]
    # ---- 1. acceleration at THIS state (Part 1, again) ----
    th, thd = s[1], s[3]

    mass = np.array([[M + m,               m * lc * np.cos(th)],
                     [m * lc * np.cos(th), I + m * lc**2]])
    rhs = np.array([F + m * lc * np.sin(th) * thd**2,
                    m * G * lc * np.sin(th)])

    xddot, thddot = np.linalg.solve(mass, rhs)

    # ---- 2. velocities: add acceleration * dt ----
    s[2] = s[2] + xddot * dt          # xdot
    s[3] = s[3] + thddot * dt         # thetadot

    # ---- 3. positions: add velocity * dt ----
    s[0] = s[0] + s[2] * dt           # x
    s[1] = s[1] + s[3] * dt           # theta

    # ---- record ----
    ts.append((step + 1) * dt)
    xs.append(s[0])
    thetas.append(s[1])

ts = np.array(ts)
xs = np.array(xs)
thetas = np.array(thetas)

# when does the pole pass 60 degrees?
fallen = np.where(np.abs(thetas) > np.deg2rad(60))[0]
t_fall = ts[fallen[0]] if len(fallen) else None

print(f"\nPart 2 -- integrated for {T} s in steps of {dt} s ({len(ts) - 1} steps):")
print(f"    theta  {np.rad2deg(thetas[0]):5.1f} deg  ->  {np.rad2deg(thetas[-1]):6.1f} deg")
print(f"    x      {xs[0]:5.2f} m    ->  {xs[-1]:6.2f} m")
if t_fall is not None:
    print(f"    passes 60 deg at t = {t_fall:.2f} s")
print("  Watch the cart: nothing pushed it, yet it moves. The falling pole")
print("  drags it -- the coupling in the off-diagonal of the mass matrix.")


# --- plot ---------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

ax1.plot(ts, np.rad2deg(thetas), lw=1.5, color="tab:red")
ax1.axhline(60, color="k", lw=0.8, ls=":", label="60 deg = fallen")
ax1.set_ylabel("theta [deg]")
ax1.set_title(f"our equations, integrated from a {np.rad2deg(0.2):.0f} deg lean, F = 0")
ax1.legend(loc="upper left", fontsize=9)
ax1.grid(alpha=0.3)

ax2.plot(ts, xs, lw=1.5, color="tab:blue")
ax2.set_ylabel("x [m]")
ax2.set_xlabel("time [s]")
ax2.set_title("the cart recoils, though nothing pushed it")
ax2.grid(alpha=0.3)

fig.tight_layout()

if SAVE:
    from pathlib import Path
    out = Path(__file__).parent / "results"
    out.mkdir(exist_ok=True)
    fig.savefig(out / "model_equations.png", dpi=110)
    print(f"\nsaved {out / 'model_equations.png'}")
else:
    plt.show()

# Try it:
#   * dt = 1/100, then 1/10. Euler drifts as the step grows -- the answer
#     depends on dt, which a correct physics never does. Refining dt is how
#     you tell integration error from a wrong model (model_test.py).
#   * F = 5.0 constant. Does a push chosen in advance keep the pole up?
#   * A number is not a check. These equations would print something just
#     as plausible with a sign wrong somewhere. That is what model_test.py
#     is for.
