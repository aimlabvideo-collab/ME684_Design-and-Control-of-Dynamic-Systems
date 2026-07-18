"""
ME 684 - Chapter 2, Lab 3: You Close the Loop

Lab 2 showed that no input chosen in advance keeps the pole up. So stop
choosing in advance: watch the pole and react.

    Left / Right arrow   push the cart
    r                    start over without waiting to fall
    q                    stop and see the plots

Drop the pole and it asks, in the terminal, whether you want another go.
Answer n and it stops and shows you the plots.

Then the question that matters:

    "What were you looking at when you decided which key to press?"

The pole. You wanted it upright, it was not, and you pressed a key based on
that difference. The difference has a name -- the error -- and reacting to
it has a name too. That is Chapter 3.

The second plot shows the force YOU applied against the angle you saw. An
arrow key is all-or-nothing, so it comes out as two bands rather than a
line -- but look at which band sits on which side. Positive angle, positive
force. You pushed toward the lean, every time, without being told to.

That is feedback on the error. Chapter 3 keeps the idea and replaces the
all-or-nothing push with one proportional to the error,

    F = -K * error

so it can push gently when the pole is nearly upright.

Read top to bottom. Same skeleton as 01c_trajectory.py: read the state,
choose a force, step, pace the viewer. Only the choice of force is new.

    python 03_keyboard_balance.py
"""

import time                          # to pace the viewer
import numpy as np                   # arrays and math
import pybullet as p                 # only to read the keyboard
import matplotlib.pyplot as plt      # the plots

from cartpole_env import CartPole


# --- settings ----------------------------------------------------------
# SPEED is not a comfort setting. The unstable pole sits at +3.97 rad/s, a
# time constant of 0.25 s, and human reaction time is about 0.2 s -- so at
# full speed you are always behind. Slowing the world down buys back the
# margin. Try SPEED = 1.0 once you can hold it at 1/3.

dt = 1.0 / 1000                 # 1000 simulation steps per second
SPEED = 1.0 / 3.0                # 1.0 = real time. Start slow.
FORCE = 10.0                     # newtons while an arrow key is held
GIVE_UP = np.deg2rad(60.0)       # past 60 degrees, call it fallen
START = np.deg2rad(2.0)          # small initial lean, so it cannot just sit


# --- the loop ----------------------------------------------------------

print("balance it!   left/right = push,   r = restart,   q = quit")

env = CartPole(gui=True, dt=dt)
env.reset([0.0, START, 0.0, 0.0])

th_hist = []                     # the angle you were looking at
F_hist = []                      # the force you applied

t = 0.0
best = 0.0
t_start = time.perf_counter()

while True:
    s = env.get_state()
    theta = s[1]

    # ---- read the keyboard ----
    keys = p.getKeyboardEvents()
    left = p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN
    right = p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN

    F = 0.0
    if right and not left:
        F = FORCE
    elif left and not right:
        F = -FORCE

    # ---- record what you did, then take the step ----
    th_hist.append(theta)
    F_hist.append(F)

    env.apply_force(F)
    env.step()
    t += dt

    # ---- quit, restart, or fall over ----
    if ord("q") in keys and keys[ord("q")] & p.KEY_WAS_TRIGGERED:
        break

    fallen = abs(theta) > GIVE_UP
    restart = ord("r") in keys and keys[ord("r")] & p.KEY_WAS_TRIGGERED

    if fallen:
        best = max(best, t)
        print(f"  fell after {t:5.2f} s   (best {best:5.2f} s)")
        # Answer in the TERMINAL, not the 3D window. The window will look
        # frozen while it waits -- that is just this prompt blocking.
        answer = input("  try again? [y/n] ").strip().lower()
        if not answer.startswith("y"):
            break

    if fallen or restart:
        env.reset([0.0, START, 0.0, 0.0])
        t = 0.0
        th_hist.clear()                  # plot the attempt you just made,
        F_hist.clear()                   # not every attempt glued together
        t_start = time.perf_counter()    # restart the clock too, and do it
                                         # AFTER the prompt, or the pacing
                                         # below counts the time you spent
                                         # answering and stops waiting at all

    # ---- hold the viewer to SPEED x real time ----
    # A plain time.sleep(dt) cannot do this: Windows rounds a sleep up to
    # about 12 ms. Compare against the wall clock instead.
    ahead = t / SPEED - (time.perf_counter() - t_start)
    if ahead > 0:
        time.sleep(ahead)

env.close()

best = max(best, t)
print(f"\nlongest balance: {best:.2f} s")


# --- what were your hands actually doing? ------------------------------

th_hist = np.rad2deg(np.array(th_hist))
F_hist = np.array(F_hist)

fig, ax = plt.subplots(1, 2, figsize=(11, 4))

ax[0].plot(np.arange(len(th_hist)) * dt, th_hist, lw=1.0)
ax[0].axhline(0, color="k", lw=0.8)
ax[0].set_xlabel("step * dt [s]")
ax[0].set_ylabel("theta [deg]")
ax[0].set_title("the angle you were watching")

ax[1].scatter(th_hist, F_hist, s=4, alpha=0.2)
ax[1].axhline(0, color="k", lw=0.8)
ax[1].axvline(0, color="k", lw=0.8)
ax[1].set_xlabel("theta [deg]")
ax[1].set_ylabel("force you applied [N]")
ax[1].set_title("your control law")

# How often did you push toward the lean? One number for the whole run.
pushed = F_hist != 0.0
agree = np.sign(F_hist[pushed]) == np.sign(th_hist[pushed])
if pushed.any():
    print(f"you pushed toward the lean {100 * agree.mean():.0f}% of the time")

for x in ax:
    x.grid(alpha=0.3)
fig.tight_layout()
plt.show()

print("\nLeaning right and pushing right is feedback on the error.")
print("Chapter 3 names it, and works out how hard to push.")
