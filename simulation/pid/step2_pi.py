"""
ME 684 - Chapter 3, Step 2: +I reaches zero error, at a price

Slide claim: the integral term stores the accumulated error, so it can hold
the arm up with the error itself at zero -- the offset that P could never
remove disappears. The bill: the loop is now third order and has a hard
stability limit. Too much Ki and it oscillates without stopping, then grows.

    python step2_pi.py            plots, in a few seconds
    python step2_pi.py --watch    also open the 3-D window (real time, slower)
    python step2_pi.py --save     headless, PNGs to results/

    tau = Kp*e + Ki * integral(e dt)

Stability (Routh, with Kd = 0):   b*Kp > J*Ki   =>   Ki < b*Kp/J = 7.2

Next: step3_pid.py adds D and buys the damping back.
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
START = np.pi             # where the arm STARTS      [rad]  (0 = straight up), (np.pi = straight down)
DURATION = 12.0
JOINT = 0                # WHICH joint (index). There is only one, so 0. NOT an angle.

KP = 18.0
KI = 5.0                 # the designed value -- safely below the limit of 7.2
KI_SWEEP = [2.0, 5.0, 7.2, 10.0]     # 7.2 = the stability limit; 10 is past it
KI_MAX = B * KP / J      # = 7.2

SAVE = "--save" in sys.argv       # headless, write PNGs (fast)
WATCH = "--watch" in sys.argv     # also open the 3-D window (real time, slower)


def run_once(kp, ki, gui=False, duration=DURATION):
    """PI control: tau = Kp*e + Ki*integral(e). Returns (t, theta)."""
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
        p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=0,
                                     cameraPitch=-10,
                                     cameraTargetPosition=[0, 0, 0.5])

    integral = 0.0
    ts, ths = [], []
    t = 0.0
    t_wall = time.perf_counter()
    while t < duration:
        theta, thetadot, _, _ = p.getJointState(robot, JOINT)

        # ---- the controller: P + I ----
        e = THETA_REF - theta
        integral += e * DT               # the running sum -- this is the "I"
        tau_ctrl = kp * e + ki * integral

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


# --- the point: does adding I shrink the steady-state error? ------------
# This plant barely damps, so BOTH P and PI ring for a long time (Step 3 is
# what fixes that). The ringing hides the steady state, so alongside the raw
# curves we draw the CENTRE of each oscillation -- a moving average over one
# period. That centre IS the steady state, and it is where "error" applies.

COMPARE_DUR = 18.0


def centre(theta_deg):
    """The middle of the wiggle: a moving average over one ~1 s period."""
    k = int(1.05 / DT)
    return np.convolve(theta_deg, np.ones(k) / k, mode="valid")


if WATCH:
    print(f"PI control, Kp={KP}, Ki={KI}. Watching the arm, then the plots.")
    run_once(KP, KI, gui=True)                      # --watch only: see the arm

tp, thp = run_once(KP, 0.0, duration=COMPARE_DUR)   # P only  (Ki = 0)
ti, thi = run_once(KP, KI,  duration=COMPARE_DUR)   # P + I
p_deg, i_deg = np.rad2deg(thp), np.rad2deg(thi)
cp, ci = centre(p_deg), centre(i_deg)
tc = tp[:len(cp)]

print(f"\n  steady-state angle = the CENTRE of the ringing:")
print(f"    P only : {cp[-1]:5.1f} deg   ({np.rad2deg(TD / KP):.0f} deg short of the 90 target)")
print(f"    P + I  : {ci[-1]:5.1f} deg   (on target -- the offset is gone)")
print("  Adding I pulled the centre onto the target. That is the whole job of I.")
print(f"  It still rings (even more than P) -- damping that is Step 3.")
print(f"  Stability limit: Ki < b*Kp/J = {KI_MAX:.1f}.")


# --- left: P vs PI centres | right: raise Ki toward the stability limit --
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))

ax1.axhline(90, color="k", lw=1.2, ls=":", label="target")
ax1.plot(tp, p_deg, color="tab:blue", lw=0.5, alpha=0.25)      # raw P (faint)
ax1.plot(ti, i_deg, color="tab:green", lw=0.5, alpha=0.25)     # raw PI (faint)
ax1.plot(tc, cp, color="tab:blue", lw=2.5,
         label=f"P only: centre -> {cp[-1]:.0f} deg (short)")
ax1.plot(tc, ci, color="tab:green", lw=2.5,
         label=f"P + I: centre -> {ci[-1]:.0f} deg (on target)")
ax1.set_xlabel("time [s]")
ax1.set_ylabel("theta [deg]")
ax1.set_title("does I remove the offset? watch the CENTRE (thick)")
ax1.legend(loc="lower right", fontsize=9)
ax1.grid(alpha=0.3)

for ki in KI_SWEEP:
    tt, tth = run_once(KP, ki, gui=False)
    label = f"Ki={ki:g}"
    if abs(ki - KI_MAX) < 1e-6:
        label += " (limit)"
    elif ki > KI_MAX:
        label += " (unstable)"
    ax2.plot(tt, np.rad2deg(tth), lw=1.0, label=label)
ax2.axhline(np.rad2deg(THETA_REF), color="k", lw=1.0, ls=":", label="target")
ax2.set_xlabel("time [s]")
ax2.set_ylabel("theta [deg]")
ax2.set_ylim(-40, 220)
ax2.set_title(f"raise Ki: at {KI_MAX:.1f} it never stops, past it it grows")
ax2.legend(loc="upper right", fontsize=9)
ax2.grid(alpha=0.3)

fig.tight_layout()

if SAVE:
    from pathlib import Path
    out = Path(__file__).parent / "results"
    out.mkdir(exist_ok=True)
    fig.savefig(out / "step2_pi.png", dpi=110)
    print(f"saved {out / 'step2_pi.png'}")
else:
    plt.show()

# Try it: Ki = 7.2 is exactly b*Kp/J, the Routh limit -- its oscillation
# neither grows nor decays. Below it the arm reaches zero error; above it
# (Ki = 10) a pole pair has crossed into the right half plane and the swing
# grows every cycle. The integrator fixed the offset but spent the stability
# margin doing it. Step 3 earns that margin back with D.
