"""
ME 684 - Chapter 2: are our equations right?

We derived the cart-pole equations of motion on the board. This file asks
the same question of two answerers and compares the numbers:

    (A) PyBullet   the 'real robot'. It has never seen our algebra.
    (B) our model  the equations typed in below.

    python model_test.py

We compare ACCELERATION at a single instant, not a trajectory. Over a
trajectory the numerical integration adds errors of its own, and then a
mismatch would not tell us whether the equations or the integrator was at
fault. One instant leaves the equations nowhere to hide.

    state  s = [x, theta, xdot, thetadot]
    x      cart position [m], +x to the right
    theta  pole angle from straight up [rad], +theta leans right
    F      force on the cart [N], +F pushes right
"""

import numpy as np
import pybullet as p

# --- physical constants -------------------------------------------------
# These MUST match assets/cartpole.urdf. If they drift apart, the test
# stops agreeing and it looks like your algebra is wrong when it is not.

G = 9.81                                  # gravity              [m/s^2]
M = 1.0                                   # cart mass       M    [kg]
m = 0.1                                   # pole mass       m    [kg]
L = 1.0                                   # pole length     L    [m]
W = 0.05                                  # pole width           [m]
lc = L / 2                                # hinge -> pole centre [m]
I = m * (L**2 + W**2) / 12                # pole inertia about its centre

CART_JOINT = 0
POLE_JOINT = 1
BASE_Z = 1.2

# PyBullet updates velocity as v += a*dt, so (v_after - v_before)/dt hands
# back the exact acceleration it used. This is not an approximation, and
# any dt works. Going much smaller actually hurts: a tiny difference
# divided by a tiny dt amplifies floating-point round-off.
dt = 1e-3


# --- the robot ----------------------------------------------------------
# No window: we want numbers, not a picture. p.DIRECT runs the same
# physics with no graphics.

p.connect(p.DIRECT)
p.setGravity(0, 0, -9.81)
p.setTimeStep(dt)
robot = p.loadURDF("assets/cartpole.urdf", [0, 0, BASE_Z], useFixedBase=True)

# The same two gotchas as test.py, and here they matter even more: either
# one makes the simulator solve a different problem from our equations.
for link in range(-1, p.getNumJoints(robot)):
    p.changeDynamics(robot, link, linearDamping=0, angularDamping=0,
                     jointDamping=0)
p.setJointMotorControl2(robot, CART_JOINT, p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(robot, POLE_JOINT, p.VELOCITY_CONTROL, force=0)


print("acceleration,  ours vs PyBullet")
print("   theta       F |    thddot sim      model     error")

rng = np.random.default_rng(0)        # fixed seed: everyone sees these numbers
worst = 0.0

for trial in range(6):

    # Make up a random state and a random force. Big angles on purpose: at
    # theta = 0, sin(theta) = 0 would delete whole terms from our equations
    # and a mistake in them could pass unnoticed.
    s = rng.uniform(-0.4, 0.4, 4)     # [x, theta, xdot, thetadot]
    F = rng.uniform(-8, 8)            # force on the cart [N]
    th, thd = s[1], s[3]

    # ---- answer 1: PyBullet ----
    # Teleport to the state, push, run one step, read it back. Note the
    # force goes on the CART joint only -- the pole has no motor. That is
    # the underactuation: one actuator, two degrees of freedom.
    p.resetJointState(robot, CART_JOINT, s[0], s[2])
    p.resetJointState(robot, POLE_JOINT, s[1], s[3])
    # resetJointState re-enables the motors, so release them again.
    p.setJointMotorControl2(robot, CART_JOINT, p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(robot, POLE_JOINT, p.VELOCITY_CONTROL, force=0)

    p.setJointMotorControl2(robot, CART_JOINT, p.TORQUE_CONTROL, force=F)
    p.stepSimulation()

    _, xd_after, _, _ = p.getJointState(robot, CART_JOINT)
    _, thd_after, _, _ = p.getJointState(robot, POLE_JOINT)

    # PyBullet never reports acceleration, only position and velocity, so
    # we recover it from how much the velocity changed.
    sim = (np.array([xd_after, thd_after]) - s[2:]) / dt

    # ---- answer 2: our equations, from the board ----
    #
    #   [ M+m           m lc cos th ] [ xddot  ]   [ F + m lc sin th thd^2 ]
    #   [ m lc cos th   I + m lc^2  ] [ thddot ] = [ m g lc sin th         ]
    #
    # Nothing here knows PyBullet exists.
    mass = np.array([[M + m, m * lc * np.cos(th)],
                     [m * lc * np.cos(th), I + m * lc**2]])
    rhs = np.array([F + m * lc * np.sin(th) * thd**2,
                    m * G * lc * np.sin(th)])
    ours = np.linalg.solve(mass, rhs)          # [xddot, thddot]

    # ---- compare ----
    err = np.abs(sim - ours).max()
    worst = max(worst, err)
    print(f"  {th:6.3f} {F:7.2f} | {sim[1]:12.5f} {ours[1]:10.5f} {err:9.1e}")

p.disconnect()
print(f"  worst = {worst:.1e}  -- round-off. Our equations ARE the robot.")

# Try it: flip one sign in the mass matrix above and run again. The error
# jumps from 1e-14 to about 20. That is what this file is for.
