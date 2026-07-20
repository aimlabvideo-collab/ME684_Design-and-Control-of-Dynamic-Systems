# Simulation

Control simulations used in lecture, written in **Python**.

| Folder | Topic |
|--------|-------|
| [`manipulator/`](manipulator/) | One-link revolute arm — writing a URDF, loading it, and commanding a joint (Ch 1) |
| [`cartpole/`](cartpole/) | Cart-pole modeling lab in PyBullet — derive the equations of motion, test open-loop inputs, then balance it by hand (Ch 2) |

Each folder has its own README with details and how to run it.

The examples use `numpy` / `scipy` / `matplotlib`, and the 3-D physics view uses
**PyBullet**.

---

## What is PyBullet, and why do we use it?

**PyBullet** is a free, open-source **physics engine** with a Python interface.
You describe a mechanical system — the masses, the shapes, and how the parts are
hinged together — and PyBullet simulates how it moves under gravity, contact, and
the forces you apply, drawing it in a 3-D window.

In this course we use it to **close the loop between theory and a "real" plant**.
In Chapter 2 we derive equations of motion by hand; in Chapters 8–9 we design a
controller from a linear model. PyBullet lets us then drop that controller onto a
full nonlinear simulator that never saw our algebra, and watch whether it
actually balances the cart-pole. It is the closest thing to hardware you can run
from a laptop, and every student will use it for the cart-pole lab.

> You do **not** give PyBullet the equations of motion. You give it a description
> of the bodies and joints (a URDF file), and it builds and solves
> $$M(q)\ddot q + C(q,\dot q)\dot q + g(q) = \tau$$ internally — the same standard
> form derived in [Chapter 2](../chapters/ch2-mathematical-models.md).

---

## Setting up Python and PyBullet — step by step

This section is written so you can **copy and paste** each command in order. If
you have never installed a Python package before, follow it line by line.

### Step 0 — Get the course code

Download individual files from GitHub, or clone the whole repo:

```bash
git clone https://github.com/aimlabvideo-collab/ME684_Design-and-Control-of-Dynamic-Systems.git
cd ME684_Design-and-Control-of-Dynamic-Systems/simulation
```

### Step 1 — Check your Python version

You need **Python 3.8, 3.9, 3.10, or 3.11**. Check with:

```bash
python --version
```

- If it prints `Python 3.8`–`3.11`, you are good.
- If the command is not found, install Python from
  [python.org/downloads](https://www.python.org/downloads/). **On Windows, tick
  "Add Python to PATH"** in the installer.
- **Avoid Python 3.12+ for now.** PyBullet's installer still uses `distutils`,
  which was removed from the standard library in 3.12, so the build there tends
  to fail.

### Step 2 — Install a C++ compiler (this is the step people miss)

Unlike most Python packages, **PyBullet ships no ready-made ("wheel") build on
PyPI — for any operating system.** Every `pip install pybullet` **compiles about
80 MB of C++ from source**, so you must have a C++ compiler installed first.
Without one, the install ends in a wall of red error text.

| Your OS | Install this first |
|---------|--------------------|
| **Windows** | [Visual Studio Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/) → run the installer → check the **"Desktop development with C++"** workload → Install. (This is a large download; do it before class.) |
| **macOS** | Open Terminal and run `xcode-select --install`. *(If you can, prefer the conda path in Step 3 — it needs no compiler.)* |
| **Linux** | `sudo apt install build-essential python3-dev` |

**The easy way out:** if you would rather not install a compiler at all, use
**conda** (Anaconda / Miniconda), which installs a prebuilt PyBullet with no
compiling — see the conda option in Step 3.

### Step 3 — Install the Python packages

**Option A — pip** (needs the compiler from Step 2):

```bash
pip install pybullet numpy scipy matplotlib
```

**Option B — conda** (no compiler needed; recommended if Step 2 gave you trouble):

```bash
conda install -c conda-forge pybullet numpy scipy matplotlib
```

> **Be patient.** With pip, the PyBullet line can sit for **several minutes with
> no output** while it compiles. That is normal — it is not frozen. Do not press
> `Ctrl-C`.

### Step 4 — Verify it works

Run this one-liner. A gray 3-D window should pop up, sit for two seconds, and
close:

```bash
python -c "import pybullet as p, time; p.connect(p.GUI); time.sleep(2); p.disconnect()"
```

If the window appears and closes cleanly, **you are done.** If not, see
[Troubleshooting](#troubleshooting).

### Step 5 — Run an example

```bash
cd manipulator
python sim.py
```

Both labs are three short scripts each, run in order and from their own
folder:

| | |
|---|---|
| [`manipulator/`](manipulator/) | `test.py` → `sim.py` → `sim_control.py` |
| [`cartpole/`](cartpole/) | `test.py` → `model_test.py` → `cart_pole_sim_control.py` |

Each folder's README says what the scripts do, the expected output, and
the questions to answer as you go.

---

## Troubleshooting

**`error: Microsoft Visual C++ 14.0 or greater is required`** (Windows)
The compiler from Step 2 is missing. Install the Visual Studio Build Tools with
the "Desktop development with C++" workload, or switch to the conda option in
Step 3.

**`pip install pybullet` looks frozen / stuck with no output**
It is compiling C++, not hanging. Give it 5–10 minutes.

**The install fails on Python 3.12 or newer**
Use Python 3.8–3.11 (Step 1), or install via conda.

**A window never appears, but there is no error**
Some remote desktops and virtual machines have no OpenGL display. Try on a normal
desktop session, or update your graphics drivers.

**`ModuleNotFoundError: No module named 'pybullet'` after installing**
You likely installed into a different Python than the one you are running.
Confirm they match:

```bash
python -c "import sys; print(sys.executable)"
pip --version
```

Make sure `pip` belongs to the same Python shown by the first command (on some
systems you need `pip3`, or `python -m pip install ...`).

**My controller / force does nothing in a script I wrote myself**
The most common PyBullet mistake: right after loading a model, each joint has a
default velocity motor enabled that cancels any torque you apply. Disable it once
with `p.setJointMotorControl2(robot, joint, p.VELOCITY_CONTROL, force=0)` before
using `TORQUE_CONTROL`.
