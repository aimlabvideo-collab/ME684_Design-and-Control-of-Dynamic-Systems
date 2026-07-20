# Chapter 1 — Robot Simulation with PyBullet

The code from the Chapter 1 slides: a one-link revolute manipulator,
from URDF to simulation.

```
test.py             does PyBullet work?
one_link_arm.urdf   the robot: base_link + link1, joined by a revolute hinge
sim.py              load it, turn gravity on, watch it fall
sim_control.py      drive the joint with a sine wave
```

Run them **from this folder** — the scripts load the URDF by bare
filename, so `python simulation/manipulator/sim.py` from elsewhere will
not find it.

```bash
cd simulation/manipulator
python test.py
python sim.py
python sim_control.py
```

Install instructions are in [`../README.md`](../README.md). Press `Ctrl-C`
in the terminal to stop a script.

## Two things the slides get wrong

Both make the arm sit motionless instead of falling, and neither prints
an error. The files here are fixed; the slides are not.

**1. The default joint motor.** Right after `loadURDF`, every joint has a
velocity motor switched on, holding it at zero speed with up to
`<limit effort="...">` of torque. Ours allows 10 N·m and gravity only
pulls with `m·g·r` = 1.0 × 9.81 × 0.25 = 2.45 N·m, so the motor wins.
Release it:

```python
p.setJointMotorControl2(robot, 0, p.VELOCITY_CONTROL, force=0)
```

This is the single most common PyBullet mistake, and it is the same one
called out in the cart-pole lab.

**2. A missing `<origin>` in `<inertial>`.** The slide gives `link1` a
mass but no inertial origin, which puts all 1 kg exactly on the rotation
axis — where gravity has no lever arm. The centre of mass of the bar is
at its middle, 0.25 m out:

```xml
<inertial>
  <origin xyz="0 0 0.25"/>     <!-- the slide omits this line -->
  <mass value="1.0"/>
  ...
```

Also worth knowing: the arm stops at 1.57 rad and stays there. That is
not physics, it is the `<limit>` in the URDF — it is lying on an end stop
90° from vertical. Change `joint1` to `type="continuous"` and drop the
`lower`/`upper` attributes if you want a pendulum that swings all the way
through.

## Assignment

Build a **two-link** manipulator. Copy `one_link_arm.urdf`, add a second
link (`link2`, a 0.4 m box) and a second revolute joint (`joint2`)
connecting `link1` → `link2`, then move both joints with
`POSITION_CONTROL` — two sine waves with different amplitudes.

Two hints. `joint2`'s `<origin>` is measured in **link1's** frame, and
link1 is 0.5 m long, so the elbow belongs at `xyz="0 0 0.5"`. And the new
joint is index **1**, so `setJointMotorControl2` gets called twice.

Submit the URDF, your script, and a short screen recording.

## Next

Chapter 2 ([`../cartpole/`](../cartpole/)) swaps `POSITION_CONTROL` for
`TORQUE_CONTROL` and stops letting PyBullet run the controller for us.
