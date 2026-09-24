"""
ME 684 - Chapter 3, Step 1 (flat version): ONE Kp, start to finish

Same experiment as step1_p.py, but written straight down the page with no
functions: pick a single Kp, run the arm once, plot where it ends up. Read
it top to bottom and every line happens in that order.

    cd simulation/pid
    python step1_2p.py

The plant (gravity modelled as a constant load, the slide's choice):

    J*thddot + b*thdot = tau - Td       J = 0.5,  b = 0.2,  Td = 5

We command the arm to horizontal (theta_ref = pi/2) and watch where it
settles. Prediction: a steady droop of  ess = Td/Kp,  below the target.
Change KP below and run it again to see the droop move.
"""

import time
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

# --- 1. the plant -------------------------------------------------------
J = 0.5                  # inertia about the joint  [kg m^2]  (set in URDF)
B = 0.2                  # joint friction           [N m s/rad]
TD = 5.0                 # constant gravity load    [N m]  -- the disturbance
DT = 1 / 500             # one simulation step      [s]
THETA_REF = np.pi / 2    # command: WHERE we want the arm  [rad]  (pi/2 = horizontal)
START = np.pi            # where the arm STARTS     [rad]  (0 = straight up, pi = straight down)
DURATION = 12.0          # how long to run          [s]
JOINT = 0                # WHICH joint (index). There is only one, so 0. NOT an angle.

# --- 2. the controller: the one number this whole file is about ---------
KP = 18.0                # <-- change me, then run again

WATCH = True            # True = open the 3-D window and run in real time (slower)

# --- 3. build the world -------------------------------------------------
p.connect(p.GUI if WATCH else p.DIRECT)
# Gravity OFF: we represent it as the constant load TD injected in the loop,
# exactly as the slides do. So the physics engine only has to integrate the
# inertia; every torque acting on the joint is one we apply ourselves.
p.setGravity(0, 0, 0)
p.setTimeStep(DT)
robot = p.loadURDF("assets/one_link_arm.urdf", useFixedBase=True,
                   flags=p.URDF_USE_INERTIA_FROM_FILE)
p.changeDynamics(robot, JOINT, linearDamping=0, angularDamping=0,
                 jointDamping=0)

# Put the arm at its START angle. resetJointState teleports it there with no
# physics -- this is how you set the initial position. Do it BEFORE releasing
# the motor below, since resetting re-enables that motor.
p.resetJointState(robot, JOINT, START)

# Release the default velocity motor. Straight after loadURDF every joint has
# one, holding it at zero speed with a large force -- it would cancel any
# torque we apply. Setting its force to zero frees the joint.
p.setJointMotorControl2(robot, JOINT, p.VELOCITY_CONTROL, force=0)

if WATCH:
    p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=0,
                                 cameraPitch=-10,
                                 cameraTargetPosition=[0, 0, 0.5])

# --- 4. run the simulation, logging as we go ----------------------------
ts = []          # time            [s]
thetas = []      # the OUTPUT: where the arm actually is  [rad]
taus = []        # the controller's torque                [N m]

t = 0.0
t_wall = time.perf_counter()

while t < DURATION:
    # where is the arm right now?
    theta, thetadot, _, _ = p.getJointState(robot, JOINT)

    # ---- the controller: these two lines are the whole of "P" ----
    e = THETA_REF - theta
    tau_ctrl = KP * e

    # ---- the plant's own torques (NOT part of the controller) ----
    #   -TD           the constant gravity load, our disturbance
    #   -B*thetadot   joint friction
    # PyBullet integrates J*thddot = (everything we apply on this line).
    p.setJointMotorControl2(robot, JOINT, p.TORQUE_CONTROL,
                            force=tau_ctrl - TD - B * thetadot)
    p.stepSimulation()

    t += DT
    ts.append(t)
    thetas.append(theta)
    taus.append(tau_ctrl)

    if WATCH:
        # Pace to the wall clock so the window runs in real time. A plain
        # time.sleep(DT) would be far too slow: Windows rounds a 2 ms sleep
        # up to ~15 ms, stretching a 12 s run into a minute.
        ahead = t - (time.perf_counter() - t_wall)
        if ahead > 0:
            time.sleep(ahead)

p.disconnect()

ts = np.array(ts)
thetas = np.array(thetas)
taus = np.array(taus)

# --- 5. measure what happened -------------------------------------------
steady = thetas[ts > DURATION - 2.0].mean()      # average over the last 2 s
ess = THETA_REF - steady                         # the droop
zeta = B / (2 * np.sqrt(J * KP))                 # damping ratio of the loop

print(f"\n  Kp          = {KP:g}")
print(f"  target      = {THETA_REF:.3f} rad  ({np.rad2deg(THETA_REF):.1f} deg)")
print(f"  settled at  = {steady:.3f} rad  ({np.rad2deg(steady):.1f} deg)")
print(f"  droop  ess  = {ess:.3f} rad   (predicted Td/Kp = {TD / KP:.3f})")
print(f"  damping zeta = {zeta:.3f}   -- tiny, so it rings for a long time")

# --- 6. plot the output --------------------------------------------------
plt.figure(figsize=(7, 4.5))
plt.plot(ts, np.rad2deg(thetas), lw=1.0, color="tab:blue", label="theta (output)")
plt.axhline(np.rad2deg(THETA_REF), color="k", lw=1.0, ls=":", label="target")
plt.axhline(np.rad2deg(steady), color="tab:red", lw=1.0, ls="--",
            label=f"settled (droop {np.rad2deg(ess):.1f} deg)")
plt.xlabel("time [s]")
plt.ylabel("theta [deg]")
plt.title(f"P only, Kp = {KP:g}: holds short of the target")
plt.legend(loc="lower right", fontsize=9)
plt.grid(alpha=0.3)
plt.tight_layout()
plt.show()

# Try it: set KP = 4.5, then 18, then 72. The droop is exactly Td/Kp --
# 1.11, 0.28, 0.07 rad. It never reaches zero for any finite Kp. And each x4
# in Kp halves the damping (zeta = b / (2*sqrt(J*Kp))), so the cure for droop
# makes the ringing worse. That dead end is why Step 2 adds the integrator.
