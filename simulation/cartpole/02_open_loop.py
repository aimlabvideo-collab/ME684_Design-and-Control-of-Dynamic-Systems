"""
ME 684 - Chapter 2, Lab 2: Open-Loop Response
=============================================

Now that we trust the model, let us poke the plant and watch.

"Open loop" means the input F(t) is decided ahead of time. It never looks at
where the pole actually is. Four inputs, none of which know anything about theta:

    zero        F = 0
    constant    F = 5 N
    sinusoid    F = 10*sin(2*pi*1.5*t)
    random      a new random force every 0.1 s

Every one of them ends with the pole on the ground. That is the point of the lab.
An unstable equilibrium cannot be held by any input chosen in advance, because
holding it requires reacting to the error you actually have -- and F(t) has no
way of knowing it.

Run:
    python 02_open_loop.py                 # GUI, runs all four in sequence
    python 02_open_loop.py sinusoid        # GUI, just one of them
    python 02_open_loop.py --save          # headless, writes PNGs to results/
"""

import sys
from pathlib import Path

import numpy as np

from cartpole_env import CartPole, plot_run, simulate

RESULTS = Path(__file__).parent / "results"

T_SIM = 4.0                       # seconds
THETA0 = np.deg2rad(1.0)          # a 1 degree nudge is all it takes
FALL_LIMIT = np.deg2rad(60.0)     # we call it "fallen" past 60 degrees


# ----------------------------------------------------------------------
# Open-loop input signals.  Note the signature: (t, s).
# They are handed the state s, and every one of them ignores it.
# ----------------------------------------------------------------------
def zero_input(t, s):
    return 0.0


def constant_input(t, s):
    return 5.0


def sinusoid_input(t, s):
    return 10.0 * np.sin(2 * np.pi * 1.5 * t)


class RandomInput:
    """Piecewise-constant random force, resampled every `hold` seconds."""

    def __init__(self, amplitude=10.0, hold=0.1, seed=0):
        self.amplitude = amplitude
        self.hold = hold
        self.rng = np.random.default_rng(seed)
        self._bin = -1
        self._value = 0.0

    def __call__(self, t, s):
        bin_idx = int(t / self.hold)
        if bin_idx != self._bin:
            self._bin = bin_idx
            self._value = self.rng.uniform(-self.amplitude, self.amplitude)
        return self._value


INPUTS = {
    "zero": (zero_input, "F = 0"),
    "constant": (constant_input, "F = 5 N"),
    "sinusoid": (sinusoid_input, "F = 10 sin(2 pi 1.5 t)"),
    "random": (RandomInput(), "F = random, resampled every 0.1 s"),
}


def time_to_fall(ts, S):
    """First time |theta| exceeds FALL_LIMIT, or None if it never does."""
    fallen = np.abs(S[:, 1]) > FALL_LIMIT
    return float(ts[np.argmax(fallen)]) if fallen.any() else None


def run_one(name, gui):
    input_fn, label = INPUTS[name]
    if isinstance(input_fn, RandomInput):
        input_fn = RandomInput()          # fresh RNG so reruns are reproducible

    print(f"\n--- {name}:  {label}")
    with CartPole(gui=gui, dt=1 / 240) as env:
        ts, S, U = simulate(env, input_fn, T_SIM, [0.0, THETA0, 0.0, 0.0])

    t_fall = time_to_fall(ts, S)
    if t_fall is None:
        print(f"    pole still up after {T_SIM} s  (|theta| stayed under 60 deg)")
    else:
        print(f"    pole passed 60 deg at t = {t_fall:.2f} s")
    print(f"    final theta = {np.rad2deg(S[-1, 1]):8.1f} deg")
    print(f"    final x     = {S[-1, 0]:8.2f} m")

    RESULTS.mkdir(exist_ok=True)
    plot_run(ts, S, U, f"Open loop: {label}",
             save=RESULTS / f"02_open_loop_{name}.png", show=gui)


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    gui = "--save" not in sys.argv

    names = args if args else list(INPUTS)
    for n in names:
        if n not in INPUTS:
            raise SystemExit(f"unknown input '{n}'. choose from: {list(INPUTS)}")

    print("=" * 70)
    print(" Open-loop response.  Starting 1 degree off vertical every time.")
    print("=" * 70)
    for n in names:
        run_one(n, gui)

    print("\n" + "=" * 70)
    print(" Takeaway")
    print("=" * 70)
    print("  Every input above was chosen in advance, without ever looking at theta.")
    print("  Every one of them drops the pole.")
    print("  To hold an unstable equilibrium the input has to depend on the state.")
    print("  Next: 03_keyboard_balance.py -- you close the loop yourself.")
