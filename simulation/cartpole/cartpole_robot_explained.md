# `cartpole_robot.py`, step by step

*What each PyBullet call does, what you pass it, and why we call it.*

---

## The idea in one slide

`cartpole_model.py` holds **our equations**. It never imports PyBullet.

`cartpole_robot.py` holds **the robot we test them against**. It is a thin
wrapper around PyBullet, a *physics engine*.

> You do not give a physics engine the equations of motion.
> You describe the **bodies and joints**, and it works out the motion itself.

That is what makes it a fair test: when the two agree to 14 decimal places,
neither can have copied the other.

The class wraps PyBullet in five operations:

```python
env = CartPole(gui=True, dt=0.001)   # build the world
env.reset(s)                         # teleport to a state
env.apply_force(F)                   # push the cart
env.step()                           # let dt seconds pass
env.get_state()                      # read the state back
```

Every lab is built from these five and nothing else.

---

## `__init__` — building the world

Eight calls, in order.

### 1. `p.connect(p.GUI)` or `p.connect(p.DIRECT)`

- **Does** — starts a physics server and returns an id for it.
- **Input** — `p.GUI` opens a 3-D window; `p.DIRECT` runs with no window.
- **Why** — `DIRECT` is much faster. Use it whenever you only want numbers.

### 2. `p.setAdditionalSearchPath(pybullet_data.getDataPath())`

- **Does** — tells PyBullet another folder to look in for model files.
- **Input** — a folder path.
- **Why** — so `loadURDF("plane.urdf")` finds the ground plane that ships
  with PyBullet, without us typing a full path.

### 3. `p.setGravity(0, 0, -9.81)`

- **Does** — sets gravity for the whole world.
- **Input** — the vector `(gx, gy, gz)` in m/s².
- **Why** — `z` is up in our URDF, so gravity points along **−z**.

### 4. `p.setTimeStep(dt)`

- **Does** — fixes how much simulated time one `stepSimulation()` advances.
- **Input** — seconds. `0.001` is 1 ms.
- **Why** — we need a known, fixed `dt` to compare against our own integrator.

### 5. `p.loadURDF(path, position, useFixedBase=True)`

- **Does** — loads a model into the world and returns an integer id.
- **Input** — the file, where to put it, and whether its base is bolted down.
- **Why** — called twice: once for the ground, once for our cart-pole.
  `useFixedBase=True` bolts the rail in place so only the cart and pole move.

> **URDF** = Unified Robot Description Format. A text file listing the rigid
> bodies (*links*), how they connect (*joints*), and each body's mass and
> inertia. Each link has three geometries: `<visual>` for drawing,
> `<collision>` for contact, `<inertial>` for the **dynamics**. Only the last
> affects the motion.
>
> We ship our own URDF because PyBullet's stock cart-pole has a placeholder
> inertia of 1.0 kg·m² — **120× too large**. With it, correct algebra would
> not match the simulator.

### 6. `p.changeDynamics(...)` — **GOTCHA 1**

```python
for link in range(-1, p.getNumJoints(self.robot)):
    p.changeDynamics(self.robot, link, linearDamping=0.0,
                     angularDamping=0.0, jointDamping=0.0)
```

- **Does** — changes physical properties of one link.
- **Input** — the robot id, the link index, and the properties to change.
  Index `-1` is the base link, which is why the loop starts there.
- **Why** — PyBullet applies its own damping by default (`angularDamping =
  0.04`). **Our equations have no friction term at all.** Leave it on and the
  simulator is solving a different problem than the one we wrote down.

*Consequence worth showing:* with damping off the system is conservative.
Released near upright it swings down, through the bottom, up the far side to
the same height, and back — forever. Energy is constant to 14 decimal places.
It is a pendulum, not a fall.

### 7. `self._release_joints()` — **GOTCHA 2**

See the next section. This is the single most common PyBullet mistake.

---

## `_release_joints` — switching off motors we never asked for

```python
p.setJointMotorControl2(self.robot, CART_JOINT, p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(self.robot, POLE_JOINT, p.VELOCITY_CONTROL, force=0)
```

### `p.setJointMotorControl2(robot, joint, mode, force=...)`

- **Does** — commands a joint's built-in motor. **Every joint in PyBullet has
  one**, because the engine is built for robot arms.
- **Input** — which robot, which joint (`0` = cart, `1` = pole), which mode,
  and a force.
- **Modes** — `POSITION_CONTROL` ("go to this angle"), `VELOCITY_CONTROL`
  ("hold this speed"), `TORQUE_CONTROL` ("apply this force").

**Why this line exists.** Straight after `loadURDF`, every joint already has a
velocity motor running, told to hold zero speed with a large force. It
silently cancels anything you apply, and your control input looks broken.

**Why `force=0` turns it off.** In `VELOCITY_CONTROL`, `force` is not a
command — it is the **maximum force the motor may spend**. Give it zero and it
can do nothing. It is a dimmer, not a switch:

| motor force budget | cart speed after one 50 N push |
|---|---|
| 0 N | 0.4878 m/s — fully released |
| 5 N | 0.4390 m/s |
| 20 N | 0.2927 m/s |
| 1000 N (≈ default) | **0.0000 m/s** — our push does nothing |

**Why `VELOCITY_CONTROL` and not something else.** Because the thing blocking
us *is* a velocity motor, and switching modes does **not** remove it:

| what we did first | then pushed with 50 N |
|---|---|
| nothing (PyBullet default) | **0.0000 m/s** |
| `TORQUE_CONTROL, force=0` — "turn it off" | **0.0000 m/s** — still blocked |
| `VELOCITY_CONTROL, force=0` | 0.4878 m/s ✓ |

That last number is worth a second look: our own mass matrix predicts
`xddot = 48.78 m/s²` for a 50 N push, so 0.01 s later the cart is doing
0.4878 m/s. Release the joints properly and the simulator agrees with the
algebra immediately.

Row two is the trap. Students reach for `TORQUE_CONTROL`, assume the old mode
is gone, and cannot see why nothing moves.

---

## `reset` — teleporting

```python
p.resetJointState(self.robot, CART_JOINT, s[0], s[2])
p.resetJointState(self.robot, POLE_JOINT, s[1], s[3])
self._release_joints()
```

### `p.resetJointState(robot, joint, position, velocity)`

- **Does** — writes a position and velocity straight into a joint.
- **Input** — which robot, which joint, the position, the velocity.
- **Why** — this is a **teleport, not a simulation**. No physics runs and no
  time passes. It puts the robot at exactly the state we want to ask about.

**Why `_release_joints()` again** — resetting a joint re-enables its motor. If
you forget this, the robot locks up after the first reset.

---

## `get_state` — reading back

```python
x,  xd,  _, _ = p.getJointState(self.robot, CART_JOINT)
th, thd, _, _ = p.getJointState(self.robot, POLE_JOINT)
```

### `p.getJointState(robot, joint)`

- **Does** — reads one joint.
- **Input** — which robot, which joint.
- **Returns** — four values: position, velocity, reaction forces, applied
  torque. We keep the first two and discard the rest.

**Note there is no acceleration.** PyBullet never reports it. When Lab 1b
needs one it has to be recovered:

```python
a = (v_after - v_before) / dt
```

State order everywhere in this chapter:

```
s = [ x , theta , xdot , thetadot ]      +x right,  +theta leans right
```

---

## `apply_force` — pushing the cart

```python
F = float(np.clip(F, -self.force_limit, self.force_limit))
p.setJointMotorControl2(self.robot, CART_JOINT, p.TORQUE_CONTROL, force=F)
```

- **Same function as above, different mode.** In `TORQUE_CONTROL`, `force` is
  the **actual force applied**, not a budget. One keyword, two meanings.
- **Why the cart only** — the pole joint gets nothing. It has no motor. One
  actuator, two degrees of freedom: the system is **underactuated**, and that
  is what makes balancing a control problem.
- `np.clip` keeps the force within a limit, as a real actuator would be.

> For a *sliding* joint, `TORQUE_CONTROL` applies a straight-line force, not a
> torque. The name is misleading.

**A force lasts exactly one step.** It is cleared after every `step()`:

| | `xdot` over four steps of 50 N |
|---|---|
| `apply_force` called once | 0.4878, 0.4878, 0.4879, 0.4881 — one kick, then coasting |
| called every step | 0.4878, 0.9756, 1.4635, 1.9514 — steady acceleration |

That is why `apply_force` sits **inside** every loop.

---

## `step` and `close`

### `p.stepSimulation()`

- **Does** — runs the physics forward by exactly `dt`.
- **Input** — none.
- **Why** — the only call in the whole file that makes time pass.

### `p.disconnect(cid)`

- **Does** — shuts the physics server down and closes the window.
- **Input** — the id from `connect`.

---

## The shape every lab has

```python
for k in range(n):
    s = env.get_state()      # look at where it is
    F = ...                  # decide what to do
    env.apply_force(F)       # push        <-- every step
    env.step()               # let time pass
```

Only the middle line ever changes:

| | how `F` is chosen |
|---|---|
| Lab 1c | `F = 0` |
| Lab 3 | your arrow keys |
| Chapter 3 | a controller |

---

## Every PyBullet call in the file

| call | what it does |
|---|---|
| `connect` / `disconnect` | start / stop the physics server |
| `setAdditionalSearchPath` | where to look for model files |
| `setGravity`, `setTimeStep` | world settings |
| `loadURDF` | put a body into the world |
| `changeDynamics` | edit a link's physical properties (**damping off**) |
| `setJointMotorControl2` | command a joint motor (**release**, and **push**) |
| `resetJointState` | teleport a joint |
| `getJointState` | read a joint |
| `stepSimulation` | advance time by `dt` |

## Three things to remember

1. **Turn the default damping off**, or the simulator solves a different
   problem than you wrote down.
2. **Release the joint motors**, or your force does nothing — and switching to
   `TORQUE_CONTROL` does *not* release them.
3. **Re-apply the force every step**, or it lasts one millisecond.
