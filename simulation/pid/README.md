# Chapter 3 — PID, one term at a time

A one-link arm held horizontal by a joint motor, built to demonstrate the
Chapter 3 slides step by step. Each script adds one term of the PID and
shows, in simulation, exactly what the slide claims.

```
test.py         meet the arm: gravity on, no controller, it falls
step1_p.py      P  only  -> holds SHORT of target (residual droop)
step2_pi.py     P + I    -> reaches zero error, but rings, with a hard limit
step3_pid.py    P + I + D -> zero error AND no ringing, together
assets/one_link_arm.urdf
```

Run from this folder. Each control script watches the arm at the designed
gains, then plots a **gain sweep** so you can see the transient change as
the gain goes up and down.

```bash
cd simulation/pid
python test.py
python step1_p.py            # or  --save  for headless PNGs to results/
python step2_pi.py
python step3_pid.py
```

## The plant

Gravity is modelled as a **constant** load `Td = mgl = 5 N·m` (its value
with the arm horizontal), the way the slides do — so the simulation matches
the slides' linear analysis instead of dragging in the `cos θ` nonlinearity.

```
J·θ̈ + b·θ̇ = τ − Td          J = 0.5,   b = 0.2,   Td = 5
```

`J` comes from the URDF; `b` (friction) and `Td` (the disturbance) are
applied in each script, so every torque on the joint is one you can see in
the code. Gravity in the engine is switched OFF and replaced by the constant
`Td` — that is the whole "treat gravity as a constant disturbance" idea.

> The URDF loads with `flags=p.URDF_USE_INERTIA_FROM_FILE`. Without it
> PyBullet ignores the `<inertia>` you wrote and recomputes it from the box
> shape, and `J` would not be 0.5.

## What each script shows

### Step 1 — P alone cannot reach zero error
Torque is bought with error: `τ = Kp·e`, and holding the arm up needs
torque, so `e` can never reach zero. The droop is exactly

```
ess = Td / Kp
```

Sweep: `Kp = 4.5, 18, 72`. Each ×4 quarters the droop (1.11 → 0.28 → 0.07
rad) but halves the damping — so the cure for droop makes the ringing worse.
It never reaches zero for any finite gain. That dead end motivates I.

### Step 2 — +I reaches zero error, at a price
The integral stores the accumulated error and holds the arm up with `e`
itself at zero. The droop vanishes. The bill: the loop is now third order
with a hard stability limit,

```
Routh:  b·Kp > J·Ki   →   Ki < 7.2
```

Sweep: `Ki = 2, 5, 7.2, 10`. At **7.2** the oscillation neither grows nor
decays (poles on the jω axis); past it (**10**) it grows every cycle. The
integrator fixed the offset but spent the stability margin doing it.

### Step 3 — +D restores the damping (the full PID)
The derivative feeds back the error's slope, `Kd·s`, which lands on the
damping coefficient of the loop — braking torque from electronics, not
friction in the hardware. The ringing of Steps 1 and 2 goes away, while
`ess = 0` survives untouched.

Sweep: `Kd = 0, 1.5, 3.4, 8`. `Kd = 0` is exactly the ringing PI of Step 2;
each increase calms the transient, and the centre stays on the target.

**A note on the numbers.** The design slide predicted `tr ≈ 0.30 s`,
`ts ≈ 1.1 s`, `P.O. ≈ 9.5%` from a pure *second-order* match that ignored
Ki. The full loop is third order, so the measured response differs — the
rise time matches (~0.3–0.4 s), but Ki adds a slow pole (a slow final creep
to zero error) and gravity opposing the upswing trims the overshoot. That
predicted-vs-measured gap is exactly what the lab slide asks you to explain:
the second-order formula is a starting point, not the finished truth.

## The two gotchas (same as everywhere in PyBullet)

Both are silent when wrong. Each script does them:

```python
# 1. remove PyBullet's built-in damping so b is only what we apply
p.changeDynamics(robot, joint, linearDamping=0, angularDamping=0, jointDamping=0)

# 2. release the default velocity motor, or it cancels our torque
p.setJointMotorControl2(robot, joint, p.VELOCITY_CONTROL, force=0)
```

## Next

The three terms here are tuned by hand. Chapter 8–9 replace the hand-tuning
with pole placement and LQR — state what you want and let the math return
the gains.
