"""
ME 684 - Chapter 3, Step 2 (flat version): ONE (Kp, Ki), start to finish

Same experiment as step2_pi.py, but written straight down the page with no
functions: pick a single Kp and Ki, run the arm once, plot where it ends up.
Read it top to bottom and every line happens in that order.

    cd simulation/pid
    python step2_2pi.py

    tau = Kp*e + Ki * integral(e dt)

The integral term stores the accumulated error, so it can hold the arm up
with the error ITSELF at zero -- the droop that P could never remove
disappears. The bill: the loop is now third order and has a hard stability
limit.

Stability (Routh, with Kd = 0):   b*Kp > J*Ki   =>   Ki < b*Kp/J = 7.2

Change KI below and run it again: 2 and 5 settle, 7.2 rings forever, 10 blows up.
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
DURATION = 18.0          # how long to run          [s]  -- long, so the centre settles
JOINT = 0                # WHICH joint (index). There is only one, so 0. NOT an angle.

# --- 2. the controller: the two numbers this whole file is about --------
KP = 10.0
KI = 5.0              # <-- change me, then run again

KI_MAX = B * KP / J      # = 7.2, the Routh stability limit
WATCH = True            # True = open the 3-D window and run in real time (slower)

print(f"  Kp = {KP:g},  Ki = {KI:g}   (stability limit Ki < b*Kp/J = {KI_MAX:.1f})")

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
integral = 0.0   # the running sum of the error -- this IS the "I". Starts at 0.

ts = []          # time            [s]
thetas = []      # the OUTPUT: where the arm actually is  [rad]
taus = []        # the controller's torque                [N m]

t = 0.0
t_wall = time.perf_counter()

while t < DURATION:
    # where is the arm right now?
    theta, thetadot, _, _ = p.getJointState(robot, JOINT)

    # ---- the controller: P + I ----
    e = THETA_REF - theta
    integral += e * DT                     # accumulate -- the only new line vs Step 1
    tau_ctrl = KP * e + KI * integral

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
        # up to ~15 ms, stretching an 18 s run into minutes.
        ahead = t - (time.perf_counter() - t_wall)
        if ahead > 0:
            time.sleep(ahead)

p.disconnect()

ts = np.array(ts)
thetas = np.array(thetas)
taus = np.array(taus)
theta_deg = np.rad2deg(thetas)

# --- 5. measure what happened -------------------------------------------
# This plant barely damps, so PI rings for a long time (Step 3 is what fixes
# that). The ringing HIDES the steady state, so we also draw the CENTRE of
# the oscillation: a moving average over one ~1.05 s period. That centre IS
# the steady state, and it is where "error" applies.
k = int(1.05 / DT)                                    # samples in one period
centre = np.convolve(theta_deg, np.ones(k) / k, mode="valid")
t_centre = ts[:len(centre)]                           # convolve shortens the array

print(f"  target            = {np.rad2deg(THETA_REF):.1f} deg")
print(f"  centre of ringing = {centre[-1]:.1f} deg")
print(f"  remaining error   = {np.rad2deg(THETA_REF) - centre[-1]:+.2f} deg")
print(f"  (P alone would sit {np.rad2deg(TD / KP):.1f} deg short -- I removed that)")

# Is the ringing dying out, holding, or growing? Compare the peak-to-peak
# swing early in the run with the swing at the end. This is the stability
# question, and the centre above cannot answer it.
n2 = int(2.0 / DT)                       # samples in 2 s
swing_early = np.ptp(theta_deg[:n2])
swing_late = np.ptp(theta_deg[-n2:])
print(f"  swing: first 2 s = {swing_early:.0f} deg  ->  last 2 s = {swing_late:.0f} deg")
if swing_late < 0.9 * swing_early:
    print("    decaying -- stable, Ki is below the limit")
elif swing_late > 1.1 * swing_early:
    print("    GROWING -- unstable, Ki is past the limit")
else:
    print("    holding -- right AT the stability limit")

# --- 6. plot the output --------------------------------------------------
plt.figure(figsize=(7, 4.5))
plt.plot(ts, theta_deg, lw=0.5, color="tab:green", alpha=0.35,
         label="theta (output, raw)")
plt.plot(t_centre, centre, lw=2.5, color="tab:green",
         label=f"centre of the ringing -> {centre[-1]:.1f} deg")
plt.axhline(np.rad2deg(THETA_REF), color="k", lw=1.2, ls=":", label="target")
plt.axhline(np.rad2deg(THETA_REF - TD / KP), color="tab:blue", lw=1.0, ls="--",
            label=f"where P alone stopped ({np.rad2deg(THETA_REF - TD / KP):.1f} deg)")
plt.xlabel("time [s]")
plt.ylabel("theta [deg]")
plt.title(f"P + I, Kp = {KP:g}, Ki = {KI:g}: the offset is gone")
plt.legend(loc="lower right", fontsize=9)
plt.grid(alpha=0.3)
plt.tight_layout()
plt.show()

# Try it: set KI = 2, then 5, then 7.2, then 10. Ki = 7.2 is exactly b*Kp/J,
# the Routh limit -- its oscillation neither grows nor decays. Below it the
# arm reaches zero error; above it (Ki = 10) a pole pair has crossed into the
# right half plane and the swing grows every cycle. The integrator fixed the
# offset but spent the stability margin doing it. Step 3 earns it back with D.
