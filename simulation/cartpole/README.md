# ME 684 — Chapter 2: Mathematical Modeling

A cart-pole simulator in PyBullet, in three labs.

By the end you will have (1) derived the equations of motion and checked them
against a physics engine that has never seen your algebra, (2) watched every
input you can choose in advance fail to keep the pole up, and (3) balanced it
yourself with the arrow keys — and then seen, in your own keypress data, that
what your fingers were doing has a name: **feedback on the error**.

That last step is the entry point to Chapter 3.

---

## 1. Install

### 1.1 Python

Use **Python 3.8 – 3.11**. Everything here was tested on 3.10.11.

> On Python 3.12+ the PyBullet build is likely to fail: its `setup.py` still
> relies on `distutils`, which was removed from the standard library in 3.12.

Check what you have:

```bash
python --version
```

### 1.2 A C++ compiler — read this before you `pip install`

PyBullet publishes **no prebuilt wheels on PyPI, for any platform**. Every
`pip install pybullet` compiles ~80 MB of C++ from source. If you do not have a
compiler, the install fails with a wall of red text.

| OS | What you need |
|---|---|
| **Windows** | [Visual Studio Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/), with the **"Desktop development with C++"** workload checked |
| **macOS** | `xcode-select --install` | (But not recommneded)
| **Linux** | `sudo apt install build-essential python3-dev` |

If you would rather not install a compiler, conda has prebuilt binaries:

```bash
conda install -c conda-forge pybullet
```

### 1.3 Install the packages

```bash
pip install pybullet numpy matplotlib
```

Expect the PyBullet step to take **several minutes** with no output. That is the
C++ compile, not a hang.

### 1.4 Check it worked

```bash
python -c "import pybullet as p; c=p.connect(p.GUI); import time; time.sleep(2); p.disconnect()"
```

A window should open, sit there for two seconds, and close. If it does, you are
done. If the window never appears but there is no error, see
[Troubleshooting](#6-troubleshooting).

---

## 2. The files

```
ME 684/
├── README.md
├── cartpole_env.py          shared: the model, the simulator wrapper, plotting
├── assets/cartpole.urdf     the robot description
├── 01a_equations.py         Lab 1a: type our equations in, solve at a point
├── 01b_check.py             Lab 1b: do they match the robot?
├── 01c_trajectory.py        Lab 1c: run both forward in time
│                           (Lab 1e, the linear model, is your assignment)
├── 02_open_loop.py          Lab 2: inputs chosen in advance all fail
├── 03_keyboard_balance.py   Lab 3: you close the loop
└── results/                 plots from Labs 2-3, plus Lab 1 reference images
```

Read `cartpole_env.py` first. Its docstring carries the full Lagrangian
derivation, the sign conventions, and the linearization — everything that goes
on the board.

### Why a custom URDF?

`pybullet_data` ships a `cartpole.urdf`, but every link in it carries a
placeholder inertia tensor of `ixx = iyy = izz = 1.0`. For our pole
(m = 0.1 kg, L = 1 m) the true inertia about the hinge axis is

```
I_yy = m(L² + w²)/12 = 0.00835 kg·m²
```

roughly **120× smaller**. With the stock file, the `(A, B)` matrices you derive
on the board do not match the simulator, and Lab 1 would "prove" your correct
derivation wrong. `assets/cartpole.urdf` fixes the inertias.

---

## 3. Lab 1 — Mathematical modeling

```bash
python 01a_equations.py     # our equations, solved at one instant
python 01b_check.py         # ... and are they right?
python 01c_trajectory.py    # run both forward, watch it in 3D
```

Three short files, run in order. Each one stands alone and reads top to
bottom -- there are no functions to jump to and no shared state between
them.

Compares two things:

| | what it is |
|---|---|
| **(A)** | PyBullet — the "real robot", which knows nothing about our algebra |
| **(B)** | our nonlinear Lagrange equations, no approximations |

**What to look for.**

**01b** evaluates accelerations from (A) and (B) at the same random states and
the same force. They agree to **~10⁻¹⁴**, i.e. to double-precision round-off.
Our equations are not an approximation of the simulator's dynamics; they are
the same equations.

**01c** releases the pole and lets it fall, integrating our own model
alongside the simulator, released from 3°. They stay together: our model made
no approximation, so there is nothing in it to degrade. (Large angles are
already covered by `01b`, which probes random states out to 0.4 rad.)

`01c` shows these plots rather than saving them. Committed here as a
reference of what you should see:

<p align="center">
  <img src="results/01_nonlinear_small.png" alt="Free fall from 3 degrees" width="520">
</p>

**01c**, second half, is the subtle one. Over a trajectory, (A) and (B) do **not** agree to
10⁻¹⁴ — they drift apart by a fraction of a degree. Part 1 already proved the
dynamics are identical, so this cannot be a modeling error. It is the
*integrator*: PyBullet uses semi-implicit Euler, we use RK4, and an unstable
plant amplifies the difference exponentially. Shrink `dt` and watch it vanish:

```
      dt |  max |A-B| deg
  0.00417 |     0.55908
  0.00100 |     0.13602
  0.00025 |     0.03412
  0.00006 |     0.00854
```

Quartering `dt` quarters the gap — heading to zero. A numerical artifact, not a
modeling error — and a distinction worth keeping, because Lab 1e produces an
error that looks just like this one in a table and behaves nothing like it.

### Lab 1e — the linear model *(assignment)*

Labs 1a-1c stopped at the nonlinear equations. Chapter 3 needs the **linearized**
model, and getting there means deliberately throwing information away:

> Linearize about the upright equilibrium by hand — `sin θ → θ`, `cos θ → 1`,
> drop the `θ̇²` term — write the result as `ṡ = As + Bu`, and measure what the
> approximation costs. Release from 3° and from 30°, plot (A), (B) and your
> linear (C) together, and report `max |A − C|` for each. Then halve `dt`
> repeatedly: one of the two errors goes to zero and the other does not.
> Which, and why?

The last question is the point of the whole lab. `|A−B|` is a numerical
artifact that refinement removes; `|A−C|` is a real approximation you chose to
make, and no amount of numerical care will remove it. **Refining `dt` cannot
fix a wrong model.**

---

## 4. Lab 2 — Open loop

```bash
python 02_open_loop.py              # GUI, runs all four inputs
python 02_open_loop.py sinusoid     # GUI, just one
python 02_open_loop.py --save       # headless, PNGs to results/
```

Four inputs, each starting 1° off vertical. None of them ever looks at where the
pole is:

| input | `F(t)` | pole passes 60° at |
|---|---|---|
| zero | `0` | 1.22 s |
| constant | `5 N` | 0.48 s |
| sinusoid | `10 sin(2π·1.5t)` | 0.46 s |
| random | resampled every 0.1 s | 0.61 s |

All four drop the pole. The signature in the code is `input_fn(t, s)` — every
one of them is handed the state `s` and every one of them ignores it.

**The point.** An unstable equilibrium cannot be held by any input chosen in
advance, no matter how clever, because holding it requires reacting to the error
you actually have, and a function of `t` alone has no way of knowing it.

---

## 5. Lab 3 — You are the controller

```bash
python 03_keyboard_balance.py
```

| key | |
|---|---|
| **←** | push the cart left (`F < 0`) |
| **→** | push the cart right (`F > 0`) |
| **r** | reset |
| **q** | quit and see the plots |

> Click on the PyBullet window first, or it will not see your keys.

**On difficulty.** The unstable pole sits at +3.97 rad/s — a 0.25 s time
constant — and human reaction time is about 0.2 s. In real time this is close to
impossible, which is why the script runs at **1/3 speed**. Edit `SPEED = 1.0`
near the top once you can hold it at a third, and feel how much of the task was
the delay rather than the physics.

`FORCE` is worth an experiment too. Raising it makes the lab **harder**, not
easier: with a reaction delay, a bigger push overshoots. Larger gain is not
better gain — a fact Chapter 3 will make precise.

### The reveal

Press `q` and two plots come up. The second is the one that matters: the force
**you** applied against the angle you were looking at.

An arrow key is all-or-nothing, so the plot is two bands rather than a line. But
notice which band sits on which side. Positive angle, positive force — you
pushed *toward* the lean, essentially every time, without being told to. The
script prints the percentage.

That is feedback, and it is the whole reason you could do what no open-loop
input in Lab 2 could:

```
error   e = θ − θ_desired = θ            (upright ⇒ θ_desired = 0)
```

Chapter 3 keeps the idea and drops the all-or-nothing part, pushing in
proportion to the error instead so it can act gently near upright:

```
F = −K · e
```

### One more thing to notice

Watch *how* you lose. Often the pole never drops at all — the **cart runs off**
while the pole stays vertical. You were watching `θ` and nothing was watching
`x`. Holding the pole up and keeping the cart in place are two objectives, and
there is only one actuator.

That is where Chapter 3 begins.

---

## 6. Troubleshooting

**`error: Microsoft Visual C++ 14.0 or greater is required`**
See §1.2. Install the Build Tools, or use conda.

**`pip install pybullet` seems frozen**
It is compiling. Give it 5–10 minutes.

**My control input does nothing.**
The single most common PyBullet mistake, and the reason `_setup_motors()` exists
in `cartpole_env.py`. Right after `loadURDF`, every joint has a velocity motor
enabled with `targetVelocity = 0` and a large max force. Unless you disable it,
that motor silently cancels whatever torque you apply:

```python
p.setJointMotorControl2(robot, joint, p.VELOCITY_CONTROL, force=0)
```

**Torque only applies for one step.**
`p.setJointMotorControl2(..., TORQUE_CONTROL, force=F)` is cleared by every
`stepSimulation()`. Re-apply it on every step.

**Arrow keys do nothing.**
Click the PyBullet window to give it focus. `p.getKeyboardEvents()` only reports
keys when its own window is focused.

**My model disagrees with the simulator.**
Two usual causes. (1) PyBullet applies `angularDamping = 0.04` by default and
your equations have no such term — `cartpole_env.py` zeroes it via
`changeDynamics`. (2) The URDF inertias are placeholders; see §2.

**Plots do not appear.**
`02_open_loop.py` takes `--save` to run headless and write PNGs to `results/`.
The other labs always show their windows.

---

## 7. What's next

Chapter 3 replaces your fingers with a controller you design on purpose:

- **PID** — pick `k_p`, `k_i`, `k_d` deliberately rather than by instinct
- **Full state feedback** — use all of `[x, θ, ẋ, θ̇]`, and finally control the
  cart position too, by placing the closed-loop poles where you want them
- **LQR** — stop guessing at pole locations; state what you care about with `Q`
  and `R` and let the Riccati equation return the gain

All three are built on the `(A, B)` you derived in Lab 1. That is why Lab 1
spent so much effort proving those matrices were right.
