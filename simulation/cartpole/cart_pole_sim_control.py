"""
ME 684 - Chapter 2: you are the controller

No input you choose in advance can keep the pole up -- try it: set F to a
constant, or a sine wave, and watch. Holding an unstable equilibrium means
reacting to the error you actually have, and a function of time alone has
no way of knowing it.

So stop choosing in advance. Watch the pole and push.

    Left / Right arrow   push the cart
    r                    start over
    q                    stop and see the plots

    python cart_pole_sim_control.py

Click the PyBullet window first, or it will not see your keys.

Then the question that matters: what were you looking at when you decided
which key to press? The pole. You wanted it upright, it was not, and you
pressed a key based on that difference. The difference has a name -- the
error -- and reacting to it has a name too. That is Chapter 3.
"""

import time
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt

DT = 1 / 240
BASE_Z = 1.2
CART_JOINT = 0
POLE_JOINT = 1

# SPEED is not a comfort setting. The unstable pole sits at +3.97 rad/s, a
# time constant of 0.25 s, and human reaction time is about 0.2 s -- so at
# full speed you are always behind. Slowing the world down buys back the
# margin. Try SPEED = 1.0 once you can hold it at 1/3.
SPEED = 1 / 3
FORCE = 10.0                      # newtons while an arrow key is held
GIVE_UP = np.deg2rad(60)          # past 60 degrees, call it fallen
START = np.deg2rad(2)             # small initial lean, so it cannot just sit


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(DT)

p.loadURDF("plane.urdf")
robot = p.loadURDF("assets/cartpole.urdf", [0, 0, BASE_Z], useFixedBase=True)

p.resetDebugVisualizerCamera(cameraDistance=4.0, cameraYaw=0,
                             cameraPitch=-15,
                             cameraTargetPosition=[0, 0, BASE_Z])

# The same two gotchas as test.py: kill PyBullet's default damping, and
# release the default velocity motors so our force actually does something.
for link in range(-1, p.getNumJoints(robot)):
    p.changeDynamics(robot, link, linearDamping=0, angularDamping=0,
                     jointDamping=0)


def reset():
    """Put the cart back at the origin with the pole slightly leaning."""
    p.resetJointState(robot, CART_JOINT, 0.0, 0.0)
    p.resetJointState(robot, POLE_JOINT, START, 0.0)
    # resetJointState re-enables the motors, so release them every time.
    p.setJointMotorControl2(robot, CART_JOINT, p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(robot, POLE_JOINT, p.VELOCITY_CONTROL, force=0)


reset()
print("balance it!   left/right = push,   r = restart,   q = quit")

th_hist = []                      # the angle you were looking at
F_hist = []                       # the force you applied
t = 0.0
best = 0.0

while True:
    th, _, _, _ = p.getJointState(robot, POLE_JOINT)

    # ---- read the keyboard ----
    keys = p.getKeyboardEvents()
    left = p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN
    right = p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN

    F = 0.0
    if right and not left:
        F = FORCE
    elif left and not right:
        F = -FORCE

    th_hist.append(th)
    F_hist.append(F)

    # ---- push, then step ----
    # The force is cleared after every step, so it has to be re-applied on
    # every single one. For a sliding joint, TORQUE_CONTROL means a
    # straight-line force.
    p.setJointMotorControl2(robot, CART_JOINT, p.TORQUE_CONTROL, force=F)
    p.stepSimulation()
    time.sleep(DT / SPEED)
    t += DT

    # ---- quit, restart, or fall over ----
    if ord("q") in keys and keys[ord("q")] & p.KEY_WAS_TRIGGERED:
        break

    fallen = abs(th) > GIVE_UP
    restart = ord("r") in keys and keys[ord("r")] & p.KEY_WAS_TRIGGERED

    if fallen:
        best = max(best, t)
        print(f"  fell after {t:5.2f} s   (best {best:5.2f} s)")

    if fallen or restart:
        reset()
        t = 0.0
        th_hist.clear()           # plot the attempt you just made,
        F_hist.clear()            # not every attempt glued together

p.disconnect()

best = max(best, t)
print(f"\nlongest balance: {best:.2f} s")


# --- what were your hands actually doing? ------------------------------
# An arrow key is all-or-nothing, so the right-hand plot comes out as two
# bands rather than a line. But look at which band sits on which side.
# Positive angle, positive force: you pushed toward the lean, essentially
# every time, without being told to.

th_hist = np.rad2deg(np.array(th_hist))
F_hist = np.array(F_hist)

fig, ax = plt.subplots(1, 2, figsize=(11, 4))

ax[0].plot(np.arange(len(th_hist)) * DT, th_hist, lw=1.0)
ax[0].axhline(0, color="k", lw=0.8)
ax[0].set_xlabel("time [s]")
ax[0].set_ylabel("theta [deg]")
ax[0].set_title("the angle you were watching")

ax[1].scatter(th_hist, F_hist, s=4, alpha=0.2)
ax[1].axhline(0, color="k", lw=0.8)
ax[1].axvline(0, color="k", lw=0.8)
ax[1].set_xlabel("theta [deg]")
ax[1].set_ylabel("force you applied [N]")
ax[1].set_title("your control law")

pushed = F_hist != 0.0
if pushed.any():
    agree = np.sign(F_hist[pushed]) == np.sign(th_hist[pushed])
    print(f"you pushed toward the lean {100 * agree.mean():.0f}% of the time")

for a in ax:
    a.grid(alpha=0.3)
fig.tight_layout()
plt.show()

print("\nLeaning right and pushing right is feedback on the error.")
print("Chapter 3 names it, and works out how hard to push.")

# One more thing to notice: often the pole never drops at all -- the CART
# runs off the rail while the pole stays vertical. You were watching theta
# and nothing was watching x. Holding the pole up and keeping the cart in
# place are two objectives, and there is only one actuator.
