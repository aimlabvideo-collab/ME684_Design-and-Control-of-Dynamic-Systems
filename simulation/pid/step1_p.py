"""
ME 684 - Chapter 3, Step 1: P alone cannot reach zero error

Slide claim: with proportional control only, the arm holds SHORT of the
target. Torque is bought with error -- tau = Kp*e -- and holding the arm up
needs torque, so the error can never reach zero. Raising Kp shrinks the gap
but never closes it, and it makes the arm ring harder.

    python step1_p.py            plots, in a few seconds
    python step1_p.py --watch    also open the 3-D window (real time, slower)
    python step1_p.py --save     headless, PNGs to results/

The plant (gravity modelled as a constant load, the slide's choice):

    J*thddot + b*thdot = tau - Td       J = 0.5,  b = 0.2,  Td = 5

We command the arm to horizontal (theta_ref = pi/2) and measure where it
settles. Prediction: a steady droop of  ess = Td/Kp,  below the target.

Next: step2_pi.py adds the integral term and kills the droop.
"""

import sys
import time
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

# --- the plant ----------------------------------------------------------
J = 0.5                  # inertia about the joint  [kg m^2]  (set in URDF)
B = 0.2                  # joint friction           [N m s/rad]
TD = 5.0                 # constant gravity load     [N m]  -- the disturbance
DT = 1 / 500
THETA_REF = np.pi / 2    # command: WHERE we want the arm  [rad]  (pi/2 = horizontal)
START = np.pi             # where the arm STARTS      [rad]  (0 = straight up), (np.pi = straight down)
DURATION = 12.0
JOINT = 0                # WHICH joint (index). There is only one, so 0. NOT an angle.

# --- the controller -----------------------------------------------------
KP = 18.0                # the designed value from the slides
KP_SWEEP = [4.5, 18.0, 72.0]     # x4 each time: watch ess drop, ringing grow

SAVE = "--save" in sys.argv       # headless, write PNGs (fast)
WATCH = "--watch" in sys.argv     # also open the 3-D window (real time, slower)


def run_once(kp, gui=False):
    """Simulate the arm under P control tau = Kp*e. Returns (t, theta)."""
    p.connect(p.GUI if gui else p.DIRECT)
    # Gravity OFF: we represent it as the constant load TD injected below,
    # exactly as the slides do. So the physics engine only has to integrate
    # the inertia; every torque acting on the joint is one we apply here.
    p.setGravity(0, 0, 0)
    p.setTimeStep(DT)
    robot = p.loadURDF("assets/one_link_arm.urdf", useFixedBase=True,
                       flags=p.URDF_USE_INERTIA_FROM_FILE)
    p.changeDynamics(robot, JOINT, linearDamping=0, angularDamping=0,
                     jointDamping=0)

    # Put the arm at its START angle. resetJointState teleports it there with
    # no physics -- this is how you set the initial position. Do it BEFORE
    # releasing the motor below, since resetting re-enables that motor.
    p.resetJointState(robot, JOINT, START)

    # Release the default velocity motor. Straight after loadURDF every joint
    # has one, holding it at zero speed with a large force -- it would cancel
    # any torque we apply. Setting its force to zero frees the joint.
    p.setJointMotorControl2(robot, JOINT, p.VELOCITY_CONTROL, force=0)

    if gui:
        p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=0,
                                     cameraPitch=-10,
                                     cameraTargetPosition=[0, 0, 0.5])

    ts, ths = [], []
    t = 0.0
    t_wall = time.perf_counter()
    while t < DURATION:
        theta, thetadot, _, _ = p.getJointState(robot, JOINT)

        # ---- the controller: this line is the whole of "P" ----
        e = THETA_REF - theta
        tau_ctrl = kp * e

        # ---- the plant's own torques (NOT part of the controller) ----
        #   -TD           the constant gravity load, our disturbance
        #   -B*thetadot   joint friction
        # PyBullet integrates J*thddot = (everything we apply here).
        p.setJointMotorControl2(robot, JOINT, p.TORQUE_CONTROL,
                                force=tau_ctrl - TD - B * thetadot)
        p.stepSimulation()
        t += DT
        ts.append(t)
        ths.append(theta)
        if gui:
            # Pace to the wall clock so the window runs in real time. A plain
            # time.sleep(DT) would be far too slow: Windows rounds a 2 ms sleep
            # up to ~15 ms, stretching a 12 s run into a minute.
            ahead = t - (time.perf_counter() - t_wall)
            if ahead > 0:
                time.sleep(ahead)

    p.disconnect()
    return np.array(ts), np.array(ths)


# --- representative run, at the designed Kp -----------------------------
# Headless by default (the plots appear in a few seconds). Add --watch to
# also see the arm move in 3-D.
t, th = run_once(KP, gui=WATCH)

steady = th[t > DURATION - 2.0].mean()     # average over the last 2 s
ess = THETA_REF - steady
zeta = B / (2 * np.sqrt(J * KP))
print(f"\n  target      = {THETA_REF:.3f} rad")
print(f"  settled at  = {steady:.3f} rad")
print(f"  droop  ess  = {ess:.3f} rad   (predicted Td/Kp = {TD / KP:.3f})")
print(f"  damping zeta = {zeta:.3f}   -- tiny, so it rings for a long time")
print("  P leaves an offset AND barely damps. Both are Step 2 and 3's job.")


# --- gain sweep: raise Kp, watch the droop shrink but never vanish -------
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))

ax1.axhline(np.rad2deg(THETA_REF), color="k", lw=1.0, ls=":", label="target")
ax1.plot(t, np.rad2deg(th), lw=1.0, color="tab:blue")
ax1.axhline(np.rad2deg(steady), color="tab:red", lw=1.0, ls="--",
            label=f"settled (droop {np.rad2deg(ess):.1f} deg)")
ax1.set_xlabel("time [s]")
ax1.set_ylabel("theta [deg]")
ax1.set_title(f"P only, Kp = {KP:g}: holds short of target")
ax1.legend(loc="lower right", fontsize=9)
ax1.grid(alpha=0.3)

for kp in KP_SWEEP:
    tt, tth = run_once(kp, gui=False)
    line, = ax2.plot(tt, np.rad2deg(tth), lw=1.0,
                     label=f"Kp={kp:g}  (droop {np.rad2deg(TD / kp):.1f} deg)")
    ax2.axhline(np.rad2deg(THETA_REF - TD / kp), color=line.get_color(),
                lw=0.8, ls="--")
ax2.axhline(np.rad2deg(THETA_REF), color="k", lw=1.0, ls=":", label="target")
ax2.set_xlabel("time [s]")
ax2.set_ylabel("theta [deg]")
ax2.set_title("higher Kp -> less droop, but more ringing")
ax2.legend(loc="lower right", fontsize=9)
ax2.grid(alpha=0.3)

fig.tight_layout()

if SAVE:
    from pathlib import Path
    out = Path(__file__).parent / "results"
    out.mkdir(exist_ok=True)
    fig.savefig(out / "step1_p.png", dpi=110)
    print(f"saved {out / 'step1_p.png'}")
else:
    plt.show()

# Try it: in KP_SWEEP the droop is exactly Td/Kp -- 1.11, 0.28, 0.07 rad for
# Kp = 4.5, 18, 72. It never reaches zero for any finite Kp. And each x4 in
# Kp halves the damping (zeta = b / (2*sqrt(J*Kp))), so the cure for droop
# makes the ringing worse. That dead end is why Step 2 adds the integrator.
