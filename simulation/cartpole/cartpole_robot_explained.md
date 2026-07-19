# Understanding `cartpole_robot.py`

*A walkthrough for students who have not written simulation code before.*

---

## 1. Why there are two files, not one

Chapter 2 has one question: **do the equations we derived on the board actually
describe a physical cart-pole?**

To answer it we need two independent things to compare. So the code is split
along exactly that line:

| file | what it is | imports PyBullet? |
|---|---|---|
| `cartpole_model.py` | the equations **we** derived | **no** |
| `cartpole_robot.py` | a **robot** to test them against | yes |

`cartpole_model.py` has never seen the simulator. That is not a stylistic
choice — it is the whole argument. When the two agree to 14 decimal places in
Lab 1b, it cannot be because one copied the other.

This document is about the second file.

---

## 2. What a physics engine is

**PyBullet** is a *physics engine*: software that computes how objects move.

The essential point, and the one students most often get backwards:

> You do **not** give PyBullet the equations of motion.
> You describe the **bodies and joints**, and it derives the motion itself.

You tell it: here is a cart of mass 1 kg that slides horizontally; here is a
pole of mass 0.1 kg hinged to it, free to rotate. From that description alone
PyBullet works out — internally, using its own methods — how the system moves
under gravity and applied forces.

That independence is what makes it a fair test of our algebra.

### Where the description lives

The description is a file: `assets/cartpole.urdf`. **URDF** stands for Unified
Robot Description Format. It is a text file listing:

- **links** — the rigid bodies (the rail, the cart, the pole)
- **joints** — how they connect, and how each is allowed to move
- for every link: its **mass**, its **inertia**, its shape

Each link carries three separate geometries, and they are not the same thing:

| tag | used for |
|---|---|
| `<visual>` | drawing on screen |
| `<collision>` | detecting contact between bodies |
| `<inertial>` | the **dynamics** — mass and how it is distributed |

Only `<inertial>` affects the motion. A body can look like a cylinder, collide
like a box, and rotate like neither, if you write it carelessly.

> **Why we ship our own URDF.** PyBullet includes a stock `cartpole.urdf`, but
> every link in it has a placeholder inertia of 1.0 kg·m². The true value for
> our pole is 0.00835 kg·m² — about **120× smaller**. With the stock file, the
> equations you derive correctly on the board would not match the simulator,
> and Lab 1 would "prove" your correct work wrong.

---

## 3. The five things you can do

The whole file exists to wrap PyBullet in five operations.

```python
env = CartPole(gui=True, dt=0.001)   # build it, open a window
env.reset(s)                         # teleport it to a state
env.apply_force(F)                   # push the cart
env.step()                           # run the physics for dt seconds
env.get_state()                      # read the state back out
```

Every lab in this chapter is built out of only these. Learn these five and you
can ignore the rest of the file.

### `CartPole(gui=..., dt=...)`

Creates the simulation.

- `gui=True` opens a 3-D window. `gui=False` runs with no window, which is much
  faster — use it when you only want numbers.
- `dt` is the **time step**: how much simulated time one call to `step()`
  advances. `dt = 0.001` means each step is one millisecond.

### `env.reset(s)`

Places the robot at the state `s = [x, θ, ẋ, θ̇]`.

This is a **teleport, not a simulation**. It writes the numbers straight into
the joints. No physics runs, no time passes. We use it to put the robot at
exactly the state we want to ask a question about.

### `env.apply_force(F)`

Pushes the cart with `F` newtons. Positive `F` pushes to the right.

Note what it does *not* do: there is no way to apply anything to the pole. The
pole joint has no motor. One actuator, two degrees of freedom — the system is
**underactuated**, and that is precisely why balancing it is a control problem.

### `env.step()`

Runs the physics engine forward by exactly `dt` seconds. This is the only call
that makes time pass.

### `env.get_state()`

Reads `[x, θ, ẋ, θ̇]` back out of the joints.

**PyBullet reports position and velocity only — never acceleration.** When
Lab 1b needs an acceleration, it has to recover it from how much the velocity
changed:

```python
a = (v_after - v_before) / dt
```

---

## 4. The state vector

Everything in this chapter passes the state around as one array of four
numbers:

```
s = [ x , theta , xdot , thetadot ]
      0     1       2       3
```

| | meaning | sign convention |
|---|---|---|
| `x` | cart position [m] | `+x` is to the right |
| `theta` | pole angle from straight up [rad] | `+θ` leans right |
| `xdot` | cart velocity [m/s] | |
| `thetadot` | pole angular velocity [rad/s] | |

`s = 0` is the upright equilibrium — and it is **unstable**.

---

## 5. Two settings that will silently ruin your day

These are the two lines in the file marked **GOTCHA**. Both cause failures that
look like a mistake in your algebra when your algebra is fine. Both are worth a
slide of their own.

### GOTCHA 1 — PyBullet adds damping you did not ask for

Straight after loading a model, PyBullet applies its own damping to every body
(`angularDamping = 0.04` by default). It is there to keep casual simulations
stable.

Our hand-derived equations contain **no friction term at all**. So unless we
switch that damping off, the simulator is solving a *different problem* than
the one we wrote down, and Lab 1b will never agree.

```python
for link in range(-1, p.getNumJoints(self.robot)):
    p.changeDynamics(self.robot, link, linearDamping=0.0,
                     angularDamping=0.0, jointDamping=0.0)
```

**Consequence to point out in class:** with damping off, the system is
*conservative*. Released from rest near upright, the pole swings down, through
the bottom, up the far side to exactly the height it started from, and comes
back — forever. It is a pendulum, not a fall. Energy is conserved to 6 decimal
places over many swings.

### GOTCHA 2 — every joint starts with a motor holding it still

This is the single most common PyBullet mistake.

Right after `loadURDF`, **every joint already has a velocity motor enabled**,
commanded to zero speed with a large maximum force. If you do not disable it,
that motor quietly cancels whatever force you apply, and your control input
appears to do nothing at all.

The fix is to set that motor's force to zero, which releases the joint:

```python
p.setJointMotorControl2(self.robot, CART_JOINT, p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(self.robot, POLE_JOINT, p.VELOCITY_CONTROL, force=0)
```

Measured, with everything else identical:

| | cart velocity after one step with `F = 50 N` |
|---|---|
| motor left on (default) | **0.000000 m/s** — the force did nothing |
| motor released | 0.4878 m/s |

Note also that `reset()` calls this again. Resetting a joint re-enables its
motor, so it has to be released every time.

---

## 6. A force lasts exactly one step

Another behaviour that surprises people:

> An applied force is **cleared after every `step()`**. It must be re-applied
> on every single step.

Measured, applying `F = 50 N` and then stepping four times:

| | `xdot` after each step |
|---|---|
| `apply_force` called **once** | 0.4878, 0.4878, 0.4879, 0.4881 |
| `apply_force` called **every step** | 0.4878, 0.9756, 1.4635, 1.9514 |

Called once, the cart gets a single kick and then coasts. Called every step, it
accelerates steadily, as a constant force should.

This is why every loop in these labs looks like this, with `apply_force` inside
the loop rather than before it:

```python
for k in range(n):
    s = env.get_state()      # look at where it is
    F = ...                  # decide what to do
    env.apply_force(F)       # push  <-- every single step
    env.step()               # let time pass
```

That four-line shape *is* the labs. In Lab 1c the force is fixed at zero; in
Lab 3 it comes from your arrow keys; in Chapter 3 it will come from a
controller. Nothing else changes.

---

## 7. Two helpers built on top

`simulate(env, input_fn, T, s0)` runs that loop for you for `T` seconds, given
a function that chooses the force. `plot_run(...)` draws cart, pole and input
against time. Lab 2 uses both; the other labs write their own loop so the shape
above stays visible.

### Real-time playback

To watch a simulation at a sensible speed you have to make the program wait.
The obvious way is wrong:

```python
time.sleep(dt)          # DON'T
```

On Windows, `sleep` is rounded up to roughly 12 ms. With `dt = 1 ms` that runs
the viewer about **12× too slow**, not at real speed. Instead, check the wall
clock and wait only for however long you are actually ahead:

```python
ahead = (k + 1) * dt - (time.perf_counter() - t_start)
if ahead > 0:
    time.sleep(ahead)
```

Measured over 2 s of simulated time at `dt = 1 ms`: `sleep(dt)` gives 25.4 s
(0.08× real time); the wall-clock version gives 2.01 s (1.00×).

---

## 8. Summary

- PyBullet derives the motion from a **description of bodies and joints**, not
  from our equations. That is what makes it an independent check.
- `cartpole_robot.py` wraps it in five calls: **construct, reset, apply_force,
  step, get_state**.
- `reset` is a teleport. `step` is the only thing that makes time pass.
- Turn PyBullet's **default damping off**, or it is solving a different
  problem than you wrote down.
- **Release the joint motors**, or your force does nothing.
- **Re-apply the force every step**, or it lasts 1 ms.
