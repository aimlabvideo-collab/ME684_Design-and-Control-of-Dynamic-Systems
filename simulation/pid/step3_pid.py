"""
ME 684 - Chapter 3, Step 3: +D restores the damping -- the full PID

Slide claim: the derivative term feeds back the error's slope, Kd*s, which
lands on the s^2 (damping) coefficient of the loop. It is braking torque
from electronics, not friction in the hardware -- so it calms the ringing
that P and I left, without reintroducing the steady offset. Zero error and
a well-damped transient stop being a trade-off.

    python step3_pid.py            plots, in a few seconds
    python step3_pid.py --watch    also open the 3-D window (real time, slower)
    python step3_pid.py --save     headless, PNGs to results/

    tau = Kp*e + Ki*integral(e) + Kd*d(e)/dt

The designed gains (from the earlier design slide, zeta = 0.6, wn = 6):

    Kp = 18,  Ki = 5,  Kd = 3.4

The second-order design aimed for a fast, well-damped rise -- tr ~ 0.3 s and
about 9.5% overshoot. Watch the arm hit both marks that P and I missed:
Step 1's droop is gone (I) and Step 2's ringing is gone (D), at the same
time. The exact overshoot and settling differ a little from the pure
second-order numbers, for reasons worth discussing -- see the note at the
bottom of the file.
"""

import sys
import time
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

J = 0.5
B = 0.2
TD = 5.0
DT = 1 / 500
THETA_REF = np.pi / 2    # command: WHERE we want the arm  [rad]  (pi/2 = horizontal)
START = 0.0              # where the arm STARTS      [rad]  (0 = straight up)
DURATION = 12.0
JOINT = 0                # WHICH joint (index). There is only one, so 0. NOT an angle.

KP = 18.0
KI = 5.0
KD = 3.4                 # the designed value
KD_SWEEP = [0.0, 1.5, 3.4, 8.0]      # 0 = the PI of Step 2 (rings)

SAVE = "--save" in sys.argv       # headless, write PNGs (fast)
WATCH = "--watch" in sys.argv     # also open the 3-D window (real time, slower)


def run_once(kp, ki, kd, gui=False):
    """Full PID. Returns (t, theta)."""
    p.connect(p.GUI if gui else p.DIRECT)
    p.setGravity(0, 0, 0)                 # gravity modelled as constant TD
    p.setTimeStep(DT)
    robot = p.loadURDF("assets/one_link_arm.urdf", useFixedBase=True,
                       flags=p.URDF_USE_INERTIA_FROM_FILE)
    p.changeDynamics(robot, JOINT, linearDamping=0, angularDamping=0,
                     jointDamping=0)
    p.resetJointState(robot, JOINT, START)     # set the start angle (before release)
    p.setJointMotorControl2(robot, JOINT, p.VELOCITY_CONTROL, force=0)
    if gui:
        p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=90,
                                     cameraPitch=-10,
                                     cameraTargetPosition=[0, 0, 0.5])

    integral = 0.0
    ts, ths = [], []
    t = 0.0
    t_wall = time.perf_counter()
    while t < DURATION:
        theta, thetadot, _, _ = p.getJointState(robot, JOINT)

        # ---- the controller: P + I + D ----
        e = THETA_REF - theta
        integral += e * DT
        # d(e)/dt = -thetadot, because the target is constant. Feeding back
        # -Kd*thetadot (the slope of the MEASUREMENT) rather than of the error
        # gives the same damping but avoids the "derivative kick" a step in
        # the target would otherwise cause.
        tau_ctrl = kp * e + ki * integral - kd * thetadot

        p.setJointMotorControl2(robot, JOINT, p.TORQUE_CONTROL,
                                force=tau_ctrl - TD - B * thetadot)
        p.stepSimulation()
        t += DT
        ts.append(t)
        ths.append(theta)
        if gui:
            # pace to the wall clock (a plain sleep(DT) is far too slow on
            # Windows, where a 2 ms sleep rounds up to ~15 ms).
            ahead = t - (time.perf_counter() - t_wall)
            if ahead > 0:
                time.sleep(ahead)

    p.disconnect()
    return np.array(ts), np.array(ths)


def metrics(t, th):
    """Rise time (10-90%), settling time (2%), percent overshoot."""
    final = THETA_REF
    step = final                                  # start from 0
    # rise time
    above10 = np.where(th >= 0.1 * step)[0]
    above90 = np.where(th >= 0.9 * step)[0]
    tr = t[above90[0]] - t[above10[0]] if len(above90) else np.nan
    # overshoot
    po = max(0.0, (th.max() - final) / step * 100)
    # settling to within 2%
    outside = np.where(np.abs(th - final) > 0.02 * step)[0]
    ts_ = t[outside[-1]] if len(outside) else 0.0
    return tr, ts_, po


# --- representative run, at the designed gains --------------------------
# Headless by default (plots in a few seconds). Add --watch to see the arm.
t, th = run_once(KP, KI, KD, gui=WATCH)

tr, ts_, po = metrics(t, th)
steady = th[t > DURATION - 1.0].mean()
print(f"\n  rise time     {tr:.2f} s     (design aimed for ~0.30 s)")
print(f"  overshoot     {po:.0f} %       (gravity opposes the rise, so it is small)")
print(f"  steady error  {THETA_REF - steady:+.3f} rad   (Step 1 left a 0.28 rad droop)")
print("  Zero error AND no ringing, together -- that is the whole point of D.")


# --- gain sweep: raise Kd, watch the ringing die ------------------------
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))

ax1.axhline(np.rad2deg(THETA_REF), color="k", lw=1.0, ls=":", label="target")
ax1.plot(t, np.rad2deg(th), lw=1.4, color="tab:purple")
ax1.set_xlabel("time [s]")
ax1.set_ylabel("theta [deg]")
ax1.set_title(f"full PID: rises in {tr:.1f}s, no ringing, reaches zero error")
ax1.legend(loc="lower right", fontsize=9)
ax1.grid(alpha=0.3)

for kd in KD_SWEEP:
    tt, tth = run_once(KP, KI, kd, gui=False)
    label = f"Kd={kd:g}"
    if kd == 0.0:
        label += " (= PI, Step 2)"
    elif kd == KD:
        label += " (designed)"
    ax2.plot(tt, np.rad2deg(tth), lw=1.2, label=label)
ax2.axhline(np.rad2deg(THETA_REF), color="k", lw=1.0, ls=":", label="target")
ax2.set_xlabel("time [s]")
ax2.set_ylabel("theta [deg]")
ax2.set_title("more Kd -> the ringing damps out (D is electronic friction)")
ax2.legend(loc="lower right", fontsize=9)
ax2.grid(alpha=0.3)

fig.tight_layout()

if SAVE:
    from pathlib import Path
    out = Path(__file__).parent / "results"
    out.mkdir(exist_ok=True)
    fig.savefig(out / "step3_pid.png", dpi=110)
    print(f"saved {out / 'step3_pid.png'}")
else:
    plt.show()

# Try it: Kd = 0 in the sweep is exactly the PI loop from Step 2 -- it rings.
# Each increase in Kd adds damping and the transient calms, while the centre
# stays on the target (D never touches the steady state, so ess = 0 survives).
# Zero error and a clean transient, together, is the whole point of the PID.
#
# Why the measured numbers differ from the design's 9.5% / 1.1 s. The design
# matched a pure SECOND-order form and, in doing so, ignored Ki. The real
# loop is THIRD order: Ki adds a slow pole near s = -0.29 that the formula
# never saw. It shows up two ways here --
#   * the integrator winds up from zero to the 5 N m it takes to hold gravity,
#     which is a slow final creep toward the target (a long 2% settling time);
#   * gravity pushes down on the way up, cancelling most of the overshoot.
# Both are real, and both are the kind of "predicted vs measured" gap the lab
# slide asks you to explain -- the second-order formula is a design starting
# point, not the finished truth. (Pre-charging the integrator to 5/Ki, or
# measuring a small step around horizontal, recovers the ~15% overshoot the
# full third-order model actually predicts.)
