"""
ME 684 - Chapter 2: the same motion, from the physics engine

model_equations.py integrated OUR equations forward in time. This file asks
PyBullet to do the same thing -- same start (0.2 rad lean, at rest), same
force (none), same 0.9 s -- and records what it sees.

    python model_simul.py            simulate, plot
    python model_simul.py --watch    also open the 3-D window
    python model_simul.py --save     headless, PNG to results/

PyBullet has never seen our matrix. You hand it the bodies and joints
(assets/cartpole.urdf) and it works out the motion itself. So if its curve
matches ours, it is not because one copied the other.

    state  s = [x, theta, xdot, thetadot]

Next: model_test.py makes that comparison exact, to 14 decimal places.
"""

import sys
import time
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt

DT = 1.0 / 1000          # same step as model_equations.py
T = 1                  # same duration
BASE_Z = 1.2             # lift the rail so the pole never reaches the floor

CART_JOINT = 0           # joint indices inside the URDF
POLE_JOINT = 1

SAVE = "--save" in sys.argv       # headless, write PNG
WATCH = "--watch" in sys.argv     # open the 3-D window
SPEED = 0.5                      # playback speed with --watch. 1.0 = real time,
                                  # 0.25 = quarter speed. The whole fall is under
                                  # a second, so slow it down to actually see it.


# --- build the world ----------------------------------------------------

p.connect(p.GUI if WATCH else p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(DT)

if WATCH:
    p.loadURDF("plane.urdf")
    p.resetDebugVisualizerCamera(cameraDistance=4.0, cameraYaw=0,
                                 cameraPitch=-15,
                                 cameraTargetPosition=[0, 0, BASE_Z])

robot = p.loadURDF("assets/cartpole.urdf", [0, 0, BASE_Z], useFixedBase=True)

# GOTCHA 1: PyBullet adds its own damping to every body by default. Our
# equations have no friction term, so switch it off or the two disagree.
for link in range(-1, p.getNumJoints(robot)):
    p.changeDynamics(robot, link, linearDamping=0, angularDamping=0,
                     jointDamping=0)

# Same start as model_equations.py: lean 0.2 rad, everything else zero.
p.resetJointState(robot, CART_JOINT, 0.0, 0.0)
p.resetJointState(robot, POLE_JOINT, 0.2, 0.0)

# GOTCHA 2: after loadURDF (and again after every resetJointState) each joint
# has a velocity motor holding it still. Release both, or nothing moves.
p.setJointMotorControl2(robot, CART_JOINT, p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(robot, POLE_JOINT, p.VELOCITY_CONTROL, force=0)

F = 0.0                  # no push, just let gravity act


# --- let the engine step it forward ------------------------------------
# This loop is the twin of the one in model_equations.py. The difference:
# there WE solved the matrix and added acceleration*dt; here stepSimulation()
# does all of that inside PyBullet, and we only read the result back.

ts = [0.0]
xs = [0.0]
thetas = [0.2]

t_wall = time.perf_counter()
for step in range(int(T / DT)):

    # the force on the cart. It is cleared after every step, so re-apply it
    # each time even though it is zero here -- this is where a controller
    # would go, in Chapter 3.
    p.setJointMotorControl2(robot, CART_JOINT, p.TORQUE_CONTROL, force=F)

    p.stepSimulation()                        # advance by exactly DT

    x, xdot, _, _ = p.getJointState(robot, CART_JOINT)
    theta, thetadot, _, _ = p.getJointState(robot, POLE_JOINT)

    ts.append((step + 1) * DT)
    xs.append(x)
    thetas.append(theta)

    if WATCH:
        # Pace the window. Simulated time ts[-1] should take ts[-1]/SPEED of
        # wall-clock time; sleep off whatever we are ahead by. (Physics is
        # untouched -- DT is the same. Only how long we wait between frames.)
        ahead = ts[-1] / SPEED - (time.perf_counter() - t_wall)
        if ahead > 0:
            time.sleep(ahead)

p.disconnect()

ts = np.array(ts)
xs = np.array(xs)
thetas = np.array(thetas)

# when does the pole pass 60 degrees?
fallen = np.where(np.abs(thetas) > np.deg2rad(60))[0]
t_fall = ts[fallen[0]] if len(fallen) else None

print(f"PyBullet -- simulated for {T} s in steps of {DT} s ({len(ts) - 1} steps):")
print(f"    theta  {np.rad2deg(thetas[0]):5.1f} deg  ->  {np.rad2deg(thetas[-1]):6.1f} deg")
print(f"    x      {xs[0]:5.2f} m    ->  {xs[-1]:6.2f} m")
if t_fall is not None:
    print(f"    passes 60 deg at t = {t_fall:.2f} s")
print("  Compare with model_equations.py. PyBullet never saw our matrix,")
print("  yet the numbers line up. That is the whole point of Chapter 2.")


# --- plot ---------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

ax1.plot(ts, np.rad2deg(thetas), lw=1.5, color="tab:red")
ax1.axhline(60, color="k", lw=0.8, ls=":", label="60 deg = fallen")
ax1.set_ylabel("theta [deg]")
ax1.set_title("PyBullet, from the same 11 deg lean, F = 0")
ax1.legend(loc="upper left", fontsize=9)
ax1.grid(alpha=0.3)

ax2.plot(ts, xs, lw=1.5, color="tab:blue")
ax2.set_ylabel("x [m]")
ax2.set_xlabel("time [s]")
ax2.set_title("the cart recoils here too")
ax2.grid(alpha=0.3)

fig.tight_layout()

if SAVE:
    from pathlib import Path
    out = Path(__file__).parent / "results"
    out.mkdir(exist_ok=True)
    fig.savefig(out / "model_simul.png", dpi=110)
    print(f"\nsaved {out / 'model_simul.png'}")
else:
    plt.show()

# Try it:
#   * Put the two plots side by side. Same shape, same 0.60 s crossing.
#     They are not identical to every decimal -- PyBullet integrates one
#     way, model_equations.py another (Euler), and integration error is
#     what differs. Shrink DT in both and watch the gap close.
#   * Comment out the changeDynamics loop. Now they DON'T agree, and no
#     amount of shrinking DT fixes it: that is a different model, not a
#     numerical error. Telling those two apart is what model_test.py is for.
