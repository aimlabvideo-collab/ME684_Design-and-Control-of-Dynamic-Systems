"""
ME 684 - Chapter 3, Step 3 (flat version): ONE (Kp, Ki, Kd), start to finish

Same experiment as step3_pid.py, but written straight down the page with no
functions: pick one set of gains, run the arm once, measure the transient.
Read it top to bottom and every line happens in that order.

    cd simulation/pid
    python step3_2pid.py

    tau = Kp*e + Ki*integral(e) - Kd*thetadot

The derivative feeds back the slope of the MEASUREMENT, not of the error.
It is braking torque from electronics, not friction in the hardware, so it
calms the ringing P and I left WITHOUT bringing back the steady offset.

Unlike Steps 1 and 2 the arm starts at theta = 0 (straight up), so this run
is a clean step from 0 to the target and the transient numbers below mean
what they usually mean.

Change KP, KI, KD and run again. step4_tuning.py predicts numbers like these
from the linear model with no physics engine -- use it to pick gains fast,
then confirm them here on the arm.
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
START = 0.0              # where the arm STARTS     [rad]  (0 = straight up)
DURATION = 12.0          # how long to run          [s]
JOINT = 0                # WHICH joint (index). There is only one, so 0. NOT an angle.

# --- 2. the controller: the three numbers this whole file is about ------
KP = 18.0
KI = 5.0
KD = 3.4                 # <-- change me, then run again

WATCH = False            # True = open the 3-D window and run in real time (slower)

print(f"  Kp = {KP:g},  Ki = {KI:g},  Kd = {KD:g}")

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
    p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=90,
                                 cameraPitch=-10,
                                 cameraTargetPosition=[0, 0, 0.5])

# --- 4. run the simulation, logging as we go ----------------------------
integral = 0.0   # the running sum of the error -- this IS the "I". Starts at 0.

ts = []          # time            [s]
thetas = []      # the OUTPUT: where the arm actually is  [rad]
taus = []        # the controller torque                  [N m]

t = 0.0
t_wall = time.perf_counter()

while t < DURATION:
    # where is the arm right now?
    theta, thetadot, _, _ = p.getJointState(robot, JOINT)

    # ---- the controller: P + I + D ----
    e = THETA_REF - theta
    integral += e * DT
    # d(e)/dt = -thetadot, because the target is constant. Feeding back
    # -Kd*thetadot (the slope of the MEASUREMENT) rather than of the error
    # gives the same damping but avoids the "derivative kick" a step in the
    # target would otherwise cause.
    tau_ctrl = KP * e + KI * integral - KD * thetadot

    # ---- the plant torques (NOT part of the controller) ----
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

# --- 5. measure the transient -------------------------------------------
# The arm starts at 0 and is commanded to THETA_REF, so the step height is
# THETA_REF itself and the usual definitions apply directly.
step = THETA_REF

# rise time: 10% of the step up to 90% of it
above10 = np.where(thetas >= 0.10 * step)[0]
above90 = np.where(thetas >= 0.90 * step)[0]
if len(above10) and len(above90):
    tr = ts[above90[0]] - ts[above10[0]]
else:
    tr = np.nan                       # never got there within DURATION

# overshoot: how far past the target the highest peak went, as a percentage
po = max(0.0, (thetas.max() - THETA_REF) / step * 100)

# settling time: the LAST moment it was still outside the 2% band
outside = np.where(np.abs(thetas - THETA_REF) > 0.02 * step)[0]
if not len(outside):
    t_settle = 0.0                    # never left the band
elif outside[-1] == len(ts) - 1:
    t_settle = np.nan                 # still outside when the run ended
else:
    t_settle = ts[outside[-1]]

steady = thetas[ts > DURATION - 1.0].mean()      # average over the last 1 s

print(f"  rise time     Tr  = {tr:6.3f} s     (10% to 90%)")
print(f"  settling time Ts  = {t_settle:6.3f} s     (2% band)"
      + ("   -- never settled within the run" if np.isnan(t_settle) else ""))
print(f"  overshoot     P.O.= {po:6.1f} %")
print(f"  steady error      = {THETA_REF - steady:+.3f} rad "
      f"({np.rad2deg(THETA_REF - steady):+.2f} deg)")
if KI > 0:
    print(f"  (Step 1 left a droop of Td/Kp = {TD / KP:.3f} rad; "
          f"the integrator is what removed it)")
else:
    print(f"  (Ki = 0, so this is PD -- the droop Td/Kp = {TD / KP:.3f} rad "
          f"is still here. Only I removes it.)")

# --- 6. plot the output --------------------------------------------------
plt.figure(figsize=(7, 4.5))
plt.plot(ts, np.rad2deg(thetas), lw=1.4, color="tab:purple", label="theta (output)")
plt.axhline(np.rad2deg(THETA_REF), color="k", lw=1.2, ls=":", label="target")

# the 2% band, and the moment it finally stayed inside
plt.axhspan(np.rad2deg(THETA_REF * 0.98), np.rad2deg(THETA_REF * 1.02),
            color="grey", alpha=0.15, label="2% band")
if not np.isnan(t_settle):
    plt.axvline(t_settle, color="tab:orange", lw=1.0, ls="--",
                label=f"Ts = {t_settle:.2f} s")

plt.xlabel("time [s]")
plt.ylabel("theta [deg]")
ts_txt = "never" if np.isnan(t_settle) else f"{t_settle:.2f} s"
plt.title(f"full PID, Kp = {KP:g}, Ki = {KI:g}, Kd = {KD:g}: "
          f"P.O. = {po:.1f} %, Ts = {ts_txt}")
plt.legend(loc="lower right", fontsize=9)
plt.grid(alpha=0.3)
plt.tight_layout()
plt.show()

# Try it: set KD = 0 and you are back to the PI loop of Step 2 -- it rings for
# the whole run. Raise KD and the ringing damps while the steady error stays
# at zero, because D never touches the steady state. Zero error AND a clean
# transient, together, is the whole point of the PID.
#
# The measured numbers sit a little off the 9.5% / 1.1 s the design slide
# aimed for, because that design matched a pure SECOND-order form and ignored
# Ki. The real loop is THIRD order: Ki adds a slow pole near s = -0.29 that
# the formula never saw, and gravity pushes down on the way up, cancelling
# part of the overshoot. step4_tuning.py drops Ki and shows the clean
# second-order case, where the textbook formulas match exactly.
