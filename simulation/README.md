# Simulation

Control simulations used in lecture, written in **Python**.

| Folder | Topic |
|--------|-------|
| [`manipulator/`](manipulator/) | One-link revolute arm — writing a URDF, loading it, and commanding a joint (Ch 1) |
| [`cartpole/`](cartpole/) | Cart-pole modeling lab in PyBullet — derive the equations of motion, test open-loop inputs, then balance it by hand (Ch 2) |
| [`pid/`](pid/) | PID one term at a time on a one-link arm — P leaves a droop, +I removes it, +D restores the damping (Ch 3) |

Each folder has its own README with details and how to run it.

The examples use `numpy` and `matplotlib`, and the 3-D physics view uses
**PyBullet**.

> **Installing for the first time?** Jump to the [install guide](#install-guide)
> and pick your computer — [macOS](#macos-install-guide) or
> [Windows](#windows-install-guide).

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

## Install guide

Everything in this course happens inside **VS Code** — writing code, running it,
and the terminal. You do not need any other application.

There are a few terminal commands, but every one of them is copy-and-paste. You
do not need to understand them yet.

**Find your computer in this table and follow only that section.**

| Your computer | Go to | Set aside |
|---|---|---|
| MacBook — any model (M1/M2/M3/M4 or Intel) | **[macOS install guide](#macos-install-guide)** | about 20 minutes |
| Windows PC or laptop | **[Windows install guide](#windows-install-guide)** | about an hour — one download is very large |

⚠️ **Windows students: start early.** One step installs Microsoft's C++ compiler,
which is a multi-gigabyte download. Do not leave it until the night before the
first lab.

Both paths end in the same place: a folder of course code that runs with one
click, and a 3-D window that opens when you run it.

### What you are installing, and why

| | What it is |
|---|---|
| **VS Code** | The editor. You will write, run, and debug all course code here. |
| **Python** | The language. Everyone in the class uses one fixed version so that every install behaves the same way. |
| **A virtual environment (`.venv`)** | A private box inside the course folder holding this course's packages, so they cannot clash with anything else on your computer. |
| **PyBullet** | The physics engine and its 3-D viewer. |
| **`numpy`, `matplotlib`** | Math and plotting. |

The two platforms differ in a couple of details:

| | macOS | Windows |
|---|---|---|
| Python version | **3.12.10** | **3.11.9** |
| PyBullet package | `pybullet-arm64` | `pybullet` |
| Extra tool needed | Xcode Command Line Tools (small) | Visual Studio Build Tools (**very large**) |

> ### Why the two platforms differ
>
> The official `pybullet` package publishes ready-made builds **for Linux only**.
> Everywhere else, `pip install pybullet` downloads about 80 MB of C++ source and
> **compiles it on your own machine**.
>
> On **Windows** that works reliably, as long as you install Microsoft's C++
> compiler first — that is Step 4 of the Windows guide, and it is the step people
> skip. Python 3.11 is used there because PyBullet's installer still relies on
> `distutils`, which was removed from Python 3.12.
>
> On **macOS** the same compile is far more fragile, especially on Apple Silicon.
> So the class uses **`pybullet-arm64`** on Macs instead: the same PyBullet,
> republished with ready-made Mac builds, which installs in about a minute and
> compiles nothing.
>
> Your code is identical either way. You always write `import pybullet as p`.

---

## macOS install guide

### Step 0 — Check your chip

Click the  **Apple menu → About This Mac** and look at **Chip**:

- **Apple M1 / M2 / M3 / M4** → Apple Silicon (most recent MacBooks)
- **Intel Core i5 / i7 / i9** → Intel Mac

Both work with this guide — Python 3.12 supports either one. You just need to
know which you have if something goes wrong later.

### Step 1 — Install VS Code

1. Go to [code.visualstudio.com](https://code.visualstudio.com/) and click
   **Download for macOS**. (The site picks the right version for your chip
   automatically.)
2. Unzip the download, then **drag `Visual Studio Code.app` into your
   Applications folder**.
   ⚠️ If you leave it in Downloads and run it from there, you will hit update
   and permission problems later.
3. Open VS Code.

**Then install the Python extension:**

1. Click the **Extensions** icon in the left sidebar (four small squares) — or
   press `Cmd + Shift + X`
2. Type `Python` in the search box
3. Find **Python** by **Microsoft** and click **Install**

### Step 2 — Open the terminal inside VS Code

Every command below goes into VS Code's built-in terminal. You never need to open
the separate Terminal app.

- Menu bar: **Terminal → New Terminal**
- Or the shortcut: **Ctrl + `** (the backtick key, top-left of the keyboard
  under `~`)

A panel opens at the bottom of the VS Code window. Type there and press Enter.

### Step 3 — Install the Xcode Command Line Tools

This gives you `git`, which you need in Step 5. Paste into the VS Code terminal
and press Enter:

```bash
xcode-select --install
```

- A popup appears — click **Install** and wait a few minutes.
- If it says `error: command line tools are already installed`, you already have
  them. Move on.

### Step 4 — Install Python 3.12.10

macOS ships with its own Python, but the version varies from machine to machine,
so the class standardizes on **3.12.10**.

⚠️ **Do not use the big Download button on the python.org front page** — it gives
you a newer version that PyBullet has no ready-made build for. Use the link
below, which downloads the exact version we want.

1. **Download:**
   [python-3.12.10-macos11.pkg](https://www.python.org/ftp/python/3.12.10/python-3.12.10-macos11.pkg)
   (43.6 MB — one file, works on both Intel and Apple Silicon)
2. Open the downloaded `.pkg` and click through with the default settings:
   **Continue → Agree → Install**
3. When it finishes, a Finder window opens. **Double-click
   `Install Certificates.command`** inside it.
   A black terminal window flashes open and closes — that is normal and means it
   worked. **Do not skip this step**: without it, `pip` fails later with an SSL
   certificate error.

Now **close the VS Code terminal and open a new one** (click the trash-can icon,
then **Terminal → New Terminal**) and check:

```bash
python3.12 --version
```

Seeing `Python 3.12.10` means it worked.

> If the link does not work, go to the
> [3.12.10 release page](https://www.python.org/downloads/release/python-31210/),
> scroll to the **Files** table at the bottom, and take
> **macOS 64-bit universal2 installer**.

### Step 5 — Download the course code

You will clone the course repository directly from inside VS Code.

1. Press `Cmd + Shift + P` to open the **Command Palette**
2. Type `Git: Clone` and press Enter
3. Paste the repository address:

   ```
   https://github.com/aimlabvideo-collab/ME684_Design-and-Control-of-Dynamic-Systems.git
   ```

4. Choose where to save it (your home folder or Documents is fine)
5. When it finishes, click **Open** on the *"Would you like to open the cloned
   repository?"* notification in the bottom right
6. If VS Code asks *"Do you trust the authors of the files in this folder?"*,
   click **Yes, I trust the authors**

The course folders should now appear in the Explorer on the left.

### Step 6 — Create the virtual environment

VS Code does this for you — no terminal commands needed.

1. `Cmd + Shift + P` → type **Python: Create Environment** → Enter
2. Choose **Venv**
3. Choose **Python 3.12.x** from the list.
   If several appear, pick the one whose path contains
   `/Library/Frameworks/Python.framework/.../3.12`
4. Wait a moment. A `.venv` folder appears in your project, and VS Code selects
   it automatically.

**To confirm it worked, use either check:**

- `Cmd + Shift + P` → **Python: Select Interpreter** → the entry
  `Python 3.12.10 ('.venv': venv)` should be marked **★ Selected**.
  *(This check works from any screen.)*
- Or open any `.py` file and look at the **far right of the thin blue Status Bar
  at the very bottom** of the VS Code window — it should read
  `3.12.10 ('.venv': venv)`.
  ⚠️ This indicator only appears when a Python file is open in the editor. On the
  Welcome tab or the terminal you will not see it.

> From now on, any terminal you open with **Terminal → New Terminal** shows
> `(.venv)` at the start of the prompt automatically. You never need to type an
> `activate` command.

### Step 7 — Install PyBullet and the other packages

Open a fresh terminal (**Terminal → New Terminal**) and check that the prompt
starts with `(.venv)`. Then:

```bash
pip install --upgrade pip
pip install pybullet-arm64 numpy matplotlib
```

- The download is about 70 MB, so it takes 1–2 minutes.
- It should install **without compiling anything**. If you instead see
  `Building wheel for pybullet ...` grinding away and then failing, your Python
  version is wrong — see [Troubleshooting](#macos-only).
- ⚠️ If `(.venv)` is **not** showing, stop. Installing without it puts the
  packages in the wrong place.

### Step 8 — Check that it works

**First, check the import.** In the `(.venv)` terminal:

```bash
python -c "import pybullet as p; print('PyBullet API version:', p.getAPIVersion())"
```

A version number means the install succeeded.

**Then open a 3-D window.** The course code already includes a test script:

```bash
cd simulation/manipulator
python test.py
```

**Success looks like:** an empty grey 3-D window opens. Click back in the
terminal and press Enter to close it.

> On macOS the 3-D window often opens **behind** VS Code. If you see nothing,
> click the Python icon (a rocket) in the Dock, or press `F3` for Mission
> Control to find it.

You are done — go to [Running the examples](#running-the-examples).

---

## Windows install guide

### Step 1 — Install VS Code

1. Go to [code.visualstudio.com](https://code.visualstudio.com/) and click
   **Download for Windows**
2. Run the downloaded installer and click through with the defaults. On the
   *Select Additional Tasks* screen, leave **"Add to PATH"** ticked.
3. Open VS Code.

**Then install the Python extension:**

1. Click the **Extensions** icon in the left sidebar (four small squares) — or
   press `Ctrl + Shift + X`
2. Type `Python` in the search box
3. Find **Python** by **Microsoft** and click **Install**

### Step 2 — Open the terminal inside VS Code

Every command below goes into VS Code's built-in terminal (a PowerShell prompt).

- Menu bar: **Terminal → New Terminal**
- Or the shortcut: **Ctrl + `** (the backtick key, top-left of the keyboard
  under `~`)

A panel opens at the bottom of the VS Code window. Type there and press Enter.

### Step 3 — Install Python 3.11.9

The class uses **Python 3.11.9** on Windows.

⚠️ **Do not use the big Download button on the python.org front page** — it
always offers the newest release. PyBullet's installer still relies on
`distutils`, which was removed from Python in version 3.12, so a newer Python
makes the install in Step 8 fail. Use the link below, which downloads the exact
version we want.

1. **Download:**
   [python-3.11.9-amd64.exe](https://www.python.org/ftp/python/3.11.9/python-3.11.9-amd64.exe)
   (25.0 MB — the 64-bit Windows installer)
2. Run it. **On the very first screen, tick the box at the bottom:**

   > ☑ **Add python.exe to PATH**

   ⚠️ **This is the single most common Windows mistake in this course.** If you
   miss it, Windows cannot find Python afterwards and nothing else in this guide
   will work. Tick it *before* clicking **Install Now**.
3. Click **Install Now** and wait for it to finish.

Now **close the VS Code terminal and open a new one** (trash-can icon, then
**Terminal → New Terminal**) and check:

```powershell
py -3.11 --version
```

Seeing `Python 3.11.9` means it worked.

> If the link does not work, go to the
> [3.11.9 release page](https://www.python.org/downloads/release/python-3119/),
> scroll to the **Files** table at the bottom, and take
> **Windows installer (64-bit)**.

### Step 4 — Install the C++ compiler

**This is the step people miss, and it is the one that takes the longest.**

Unlike most Python packages, PyBullet has no ready-made build for Windows.
`pip install pybullet` in Step 8 will **compile about 80 MB of C++ source on your
PC**, so the compiler has to be in place first. Without it, the install ends in a
wall of red error text.

1. Go to
   [Visual Studio Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)
   and click **Download Build Tools**
2. Run the downloaded installer
3. In the workload list, tick **"Desktop development with C++"**
4. Click **Install** and wait

⚠️ This is a **multi-gigabyte download** and can take a long time on a slow
connection. Start it well before the first lab. You can leave it running and come
back.

### Step 5 — Install Git

Unlike macOS, Windows does not come with Git, and you need it for the next step.

1. Download from [git-scm.com/download/win](https://git-scm.com/download/win)
   (the download starts on its own)
2. Run the installer and accept every default — just keep clicking **Next**,
   then **Install**
3. **Close VS Code completely and reopen it** so it picks up the new Git

### Step 6 — Download the course code

1. Press `Ctrl + Shift + P` to open the **Command Palette**
2. Type `Git: Clone` and press Enter
3. Paste the repository address:

   ```
   https://github.com/aimlabvideo-collab/ME684_Design-and-Control-of-Dynamic-Systems.git
   ```

4. Choose where to save it (Documents is fine — avoid OneDrive-synced folders if
   you can, since syncing can lock files while Python is using them)
5. When it finishes, click **Open** on the *"Would you like to open the cloned
   repository?"* notification in the bottom right
6. If VS Code asks *"Do you trust the authors of the files in this folder?"*,
   click **Yes, I trust the authors**

The course folders should now appear in the Explorer on the left.

### Step 7 — Create the virtual environment

VS Code does this for you — no terminal commands needed.

1. `Ctrl + Shift + P` → type **Python: Create Environment** → Enter
2. Choose **Venv**
3. Choose **Python 3.11.x** from the list.
   If several appear, pick the one whose path contains
   `AppData\Local\Programs\Python\Python311`
4. Wait a moment. A `.venv` folder appears in your project, and VS Code selects
   it automatically.

**To confirm it worked, use either check:**

- `Ctrl + Shift + P` → **Python: Select Interpreter** → the entry
  `Python 3.11.9 ('.venv': venv)` should be marked **★ Selected**.
  *(This check works from any screen.)*
- Or open any `.py` file and look at the **far right of the thin blue Status Bar
  at the very bottom** of the VS Code window — it should read
  `3.11.9 ('.venv': venv)`.
  ⚠️ This indicator only appears when a Python file is open in the editor.

> From now on, any terminal you open with **Terminal → New Terminal** shows
> `(.venv)` at the start of the prompt automatically. You never need to type an
> `activate` command.

### Step 8 — Install PyBullet and the other packages

Open a fresh terminal (**Terminal → New Terminal**) and check that the prompt
starts with `(.venv)`. Then:

```powershell
pip install --upgrade pip
pip install pybullet numpy matplotlib
```

> ### Be patient — this one is slow
>
> The `pybullet` line can sit for **five to ten minutes with no output at all**
> while it compiles the C++ from Step 4. **That is normal. It is not frozen. Do
> not press `Ctrl-C`.**

- ⚠️ If `(.venv)` is **not** showing in the prompt, stop. Installing without it
  puts the packages in the wrong place.
- If it ends in red text about `Microsoft Visual C++ 14.0 or greater`, Step 4 did
  not finish — see [Troubleshooting](#windows-only).

### Step 9 — Check that it works

**First, check the import.** In the `(.venv)` terminal:

```powershell
python -c "import pybullet as p; print('PyBullet API version:', p.getAPIVersion())"
```

A version number means the install succeeded.

**Then open a 3-D window.** The course code already includes a test script:

```powershell
cd simulation\manipulator
python test.py
```

**Success looks like:** an empty grey 3-D window opens. Click back in the
terminal and press Enter to close it.

You are done — go to [Running the examples](#running-the-examples).

---

## Running the examples

Every script must be run **from its own folder**, because the scripts load their
URDF files by bare filename. Running `python simulation/manipulator/sim.py` from
somewhere else will not find the robot.

```bash
cd simulation/manipulator
python sim.py
```

You can also click the **▶ Run Python File** button at the top right of VS Code
with the file open — but first make sure the terminal is in the right folder.

Each lab is a short sequence of scripts, run in order:

| Lab | Run in this order |
|---|---|
| [`manipulator/`](manipulator/) | `test.py` → `sim.py` → `sim_control.py` |
| [`cartpole/`](cartpole/) | `test.py` → `model_test.py` → `cart_pole_sim_control.py` |
| [`pid/`](pid/) | `test.py` → `step1_p.py` → `step2_pi.py` → `step3_pid.py` |

Each folder's README says what the scripts do, the expected output, and the
questions to answer as you go.

Press **`Ctrl-C`** in the terminal to stop a script that loops forever.

---

## Troubleshooting

### Both platforms

**`ModuleNotFoundError: No module named 'pybullet'` when you press ▶**
VS Code is running a different Python than the one you installed into.
Press `Cmd/Ctrl + Shift + P` → **Python: Select Interpreter** → choose the entry
marked `('.venv': venv)`. Then close the terminal with the trash-can icon, open a
new one, and try again.

**The terminal prompt does not show `(.venv)`**
Select the interpreter as above, then **close the existing terminal and open a
new one**. Existing terminals do not update.

**A 3-D window never appears, but there is no error message**
Some remote desktops and virtual machines have no OpenGL display. Run on a
normal desktop session, or update your graphics drivers.

**The 3-D window opens and closes immediately**
The window closes when the script ends. Check that the simulation loop with
`p.stepSimulation()` and `time.sleep()` is still running — or that the script
ends in `input("Press Enter to quit")`.

**`(base)` keeps appearing in the terminal**
This one only affects students who already have Anaconda or Miniconda installed
from another course. You do **not** need it for ME 684 — follow the guide above
as written, and do not install it.

`(base)` is not an error. It just means Anaconda switches itself on in every
terminal. Run `which python` (macOS) or `where python` (Windows):

- If the path ends in `.venv/bin/python` or `.venv\Scripts\python.exe`, you are
  fine — `(.venv) (base)` side by side still uses the right Python.
- If it shows an Anaconda path instead, press `Cmd/Ctrl + Shift + P` →
  **Python: Select Interpreter**, choose the `('.venv': venv)` entry, then close
  the terminal and open a new one.

To stop `(base)` from appearing at all:
`conda config --set auto_activate_base false`

**Your own script does nothing when you apply a force or torque**
The most common PyBullet mistake: right after loading a model, every joint has a
default velocity motor enabled that silently cancels any torque you apply.
Disable it once per joint before using `TORQUE_CONTROL`:

```python
p.setJointMotorControl2(robot, joint, p.VELOCITY_CONTROL, force=0)
```

### macOS only

**`pip install` fails with `SSL: CERTIFICATE_VERIFY_FAILED`**
You skipped `Install Certificates.command` in Step 4. Open Finder →
**Applications → Python 3.12**, double-click `Install Certificates.command`, wait
for the black window to close, then try the install again.

**The install starts compiling and fails (`Building wheel for pybullet ...`)**
On a Mac this should never happen — `pybullet-arm64` ships ready-made builds. It
means your Python version has no matching build:

| Python | Apple Silicon | Intel Mac |
|---|---|---|
| 3.10 | ✅ macOS 11+ | ❌ |
| 3.11 | ✅ macOS 10.9+ | ✅ |
| **3.12 — use this one** | ✅ macOS 10.13+ | ✅ |
| 3.13 | ✅ macOS 14 Sonoma or newer | ❌ |
| 3.14 | ❌ | ❌ |

Delete the `.venv` folder in the Explorer, then redo **Step 6** and pick
**3.12** this time.

**`zsh: command not found: python`**
The virtual environment is not active. Check the two items at the top of this
section. Outside a virtual environment you must type `python3.12`, not `python`.

**The 3-D window is nowhere to be seen**
It usually opens **behind** VS Code. Click the Python icon (a rocket) in the
Dock, or press `F3` for Mission Control.

**Note on writing your own scripts:** on macOS, `p.connect(p.GUI)` must be called
from the **main thread**. All course examples already do this.

### Windows only

**`error: Microsoft Visual C++ 14.0 or greater is required`**
The compiler from Step 4 is missing, or its install did not finish. Run the
Visual Studio Build Tools installer again, make sure the **"Desktop development
with C++"** workload is ticked, and let it finish completely. Then restart your
PC and try Step 8 again.
If it still fails after that, bring your laptop to office hours rather than
trying other install methods found online.

**`pip install pybullet` looks frozen, with no output for minutes**
It is compiling C++, not hanging. Give it 5–10 minutes and do not press
`Ctrl-C`.

**The install fails and you are on Python 3.12 or newer**
PyBullet's installer needs `distutils`, which newer Python versions removed.
Delete the `.venv` folder in the Explorer, then redo **Step 7** and choose
**Python 3.11** this time. If 3.11 is not in the list, redo Step 3.

**`'python' is not recognized as an internal or external command`**
The **Add python.exe to PATH** box was not ticked in Step 3. The easiest fix is
to run the installer again, choose **Modify**, and make sure PATH is included —
or simply reinstall with the box ticked.

**`cannot be loaded because running scripts is disabled on this system`**
PowerShell is blocking the environment activation script. Open a terminal and
run this once, then open a new terminal:

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

**`Git: Clone` does not appear in the Command Palette**
Git is not installed, or VS Code has not noticed it yet. Do Step 5, then close
and reopen VS Code completely.

**Antivirus or Controlled Folder Access blocks the install**
Windows Security can block writes into `Documents` or `Desktop`. Clone the course
folder somewhere like `C:\Users\<you>\ME684` instead.
