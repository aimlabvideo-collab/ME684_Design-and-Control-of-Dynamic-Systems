"""
ME 684 - Chapter 2, Lab 3: You Are The Controller
=================================================

Lab 2 showed that no input picked in advance can hold the pole up.
So now *you* hold it up. Arrow keys push the cart left and right.

    LEFT  arrow   push the cart left   (F < 0)
    RIGHT arrow   push the cart right  (F > 0)
    r             reset
    q             quit and analyze

Then, afterwards, the important question:

    "What were you looking at when you decided which key to press?"

You were looking at the pole. You wanted it upright and it was not, and you
pressed a key based on that difference. That difference has a name:

    error   e = theta - theta_desired ,   theta_desired = 0

This script records every key you press together with the state of the pole at
that instant. When you quit, it fits a line through your data and shows you the
control law your fingers were running the whole time.

Note on difficulty: the unstable pole of this plant sits at +3.97 rad/s, a time
constant of 0.25 s. Human reaction time is around 0.2 s, which is why this is
hard in real time. The simulation runs at 1/3 speed by default so that you have
a fighting chance. Try --slow 1 once you can do it.

Run:
    python 03_keyboard_balance.py
    python 03_keyboard_balance.py --slow 1        # real time, much harder
    python 03_keyboard_balance.py --force 10      # stronger push (not easier!)
"""

import argparse
import time
from collections import deque
from pathlib import Path

import numpy as np
import pybullet as p

from cartpole_env import CartPole, BASE_Z

RESULTS = Path(__file__).parent / "results"

FALL_ANGLE = np.deg2rad(60.0)     # episode over past this angle
X_LIMIT = 4.0                     # episode over if the cart runs off screen
THETA0 = np.deg2rad(1.0)          # small nudge to start


class Episode:
    """One attempt. Records (theta, thetadot, u) at every step."""

    def __init__(self):
        self.theta, self.thetadot, self.u = [], [], []
        self.t_end = 0.0

    def log(self, s, u, t):
        self.theta.append(s[1])
        self.thetadot.append(s[3])
        self.u.append(u)
        self.t_end = t


class KeyboardOperator:
    """A human at the arrow keys."""

    def __init__(self, force):
        self.force = force

    def __call__(self, s, t):
        """Poll the keyboard. Returns (F, reset_pressed, quit_pressed)."""
        keys = p.getKeyboardEvents()

        def held(k):
            return k in keys and keys[k] & p.KEY_IS_DOWN

        left = held(p.B3G_LEFT_ARROW)
        right = held(p.B3G_RIGHT_ARROW)

        F = 0.0
        if right and not left:
            F = self.force
        elif left and not right:
            F = -self.force

        reset = ord("r") in keys and keys[ord("r")] & p.KEY_WAS_TRIGGERED
        quit_ = ord("q") in keys and keys[ord("q")] & p.KEY_WAS_TRIGGERED
        return F, reset, quit_


class AutoOperator:
    """A stand-in for a human: bang-bang on the error, with a reaction delay.

    Useful for demoing without touching the keyboard, and it makes the point that
    a human is just a (rather noisy, rather slow) feedback controller.

    The delay is what makes this hard. Raise it past ~0.15 s of simulated time and
    no choice of force keeps the pole up -- which is exactly why the lab runs in
    slow motion.
    """

    def __init__(self, force, dt, reaction_s=0.20, slow=1.0,
                 k_thetadot=0.25, deadzone=0.005, hold_s=0.25, quit_after=25.0):
        """reaction_s and hold_s are WALL-CLOCK times, the way a person experiences
        them. Slowing the simulation down by `slow` shrinks both of them measured in
        simulated seconds -- which is precisely why slow motion makes this winnable.
        """
        self.force = force
        self.k = k_thetadot
        self.deadzone = deadzone
        self.quit_after = quit_after
        self.elapsed = 0.0
        self.dt = dt

        # A person does not re-decide 240 times a second. They commit to a key for
        # a beat before reconsidering. Without this the operator chatters at the
        # step rate and stops resembling anything a hand could do.
        self.hold_steps = max(1, int((hold_s / slow) / dt))
        self._k = 0
        self._F = 0.0

        n = max(1, int((reaction_s / slow) / dt))
        self.buf = deque([np.zeros(4)] * n, maxlen=n)

    def __call__(self, s, t):
        self.elapsed += self.dt
        self.buf.append(np.asarray(s, dtype=float))

        if self._k % self.hold_steps == 0:
            seen = self.buf[0]                    # the state as it was `reaction` ago
            signal = seen[1] + self.k * seen[3]   # theta + k * thetadot
            if signal > self.deadzone:
                self._F = self.force              # leaning right -> push right
            elif signal < -self.deadzone:
                self._F = -self.force
            else:
                self._F = 0.0
        self._k += 1
        return self._F, False, self.elapsed > self.quit_after


def hud(text_id, s, t, best):
    """Draw the current error on screen, so the idea of 'error' is visible."""
    e_deg = np.rad2deg(s[1])
    arrow = "<-- press LEFT" if e_deg < -0.5 else (
        "press RIGHT -->" if e_deg > 0.5 else "balanced")
    msg = (f"t = {t:5.2f} s   (best {best:5.2f} s)    "
           f"error e = theta - 0 = {e_deg:+6.2f} deg    {arrow}")
    return p.addUserDebugText(
        msg, [0, 0, BASE_Z + 1.4], textColorRGB=[0, 0, 0], textSize=1.3,
        replaceItemUniqueId=text_id,
    )


def run(args):
    gui = not args.auto or args.watch
    dt = 1 / 240
    env = CartPole(gui=gui, dt=dt, force_limit=args.force)

    if args.auto:
        operator = AutoOperator(args.force, dt, slow=args.slow)
        print("=" * 70)
        print(" AUTO mode: a bang-bang operator with a 0.20 s reaction time plays.")
        print("=" * 70)
    else:
        operator = KeyboardOperator(args.force)
        print("=" * 70)
        print(" Balance the pole.   LEFT / RIGHT arrows.   r = reset,  q = quit")
        print(f" Force per keypress: {args.force} N"
              f"     Speed: {1/args.slow:.2f}x real time")
        print("=" * 70)
        print(" Click on the PyBullet window first, or it will not see your keys.\n")

    episodes = []
    running = True
    best = 0.0
    text_id = -1

    while running:
        env.reset([0.0, THETA0, 0.0, 0.0])
        ep = Episode()
        k = 0

        while True:
            t = k * env.dt
            s = env.get_state()

            F, reset, quit_ = operator(s, t)
            if quit_:
                running = False
                break
            if reset:
                break

            u = env.apply_force(F)
            env.step()
            ep.log(s, u, t)

            if gui and k % 8 == 0:
                text_id = hud(text_id, s, t, best)

            if abs(s[1]) > FALL_ANGLE or abs(s[0]) > X_LIMIT:
                reason = "pole fell" if abs(s[1]) > FALL_ANGLE else "cart ran away"
                best = max(best, t)
                print(f"  {reason:14s} survived {t:5.2f} s      (best {best:5.2f} s)")
                if gui:
                    time.sleep(0.7)
                break

            k += 1
            if gui:
                time.sleep(env.dt * args.slow)

        if ep.u:
            episodes.append(ep)
        if args.auto and not running:
            best = max(best, ep.t_end)
            print(f"  auto operator session over, best run {best:5.2f} s")

    env.close()
    return episodes


# ----------------------------------------------------------------------
# The reveal: what control law were your fingers running?
# ----------------------------------------------------------------------
DT = 1.0 / 240.0


def rule_accuracy(theta, thetadot, u, delay_steps, tau):
    """How often does  sign(theta(t-d) + tau*thetadot(t-d))  match the key at time t?

    Note the delay. You cannot react to what you have not seen yet.
    """
    if delay_steps == 0:
        th, thd, key = theta, thetadot, u
    else:
        th = theta[:-delay_steps]
        thd = thetadot[:-delay_steps]
        key = u[delay_steps:]

    m = key != 0
    if m.sum() < 20:
        return np.nan
    sigma = th[m] + tau * thd[m]
    return float((np.sign(sigma) == np.sign(key[m])).mean())


def fit_policy(theta, thetadot, u, max_delay_s=0.35):
    """Recover the reaction delay and the derivative ratio your hand was using.

    Why sweep the delay instead of just regressing on the current state?

    Because this is a CLOSED LOOP. The state you see now is partly a consequence
    of the key you are pressing now, so correlating the two is circular: a naive
    fit on the current state reports "you only used theta, no derivative at all",
    and that conclusion is an artifact, not a finding. The honest question is
    which past state explains the present key. Sweeping the delay asks exactly
    that, and it recovers the derivative term that the naive fit destroys.

    Returns (best_delay_s, best_tau, best_acc, acc_naive).
    """
    delays = np.arange(0, int(max_delay_s / DT), 4)      # every ~17 ms
    taus = np.concatenate([[0.0], np.linspace(0.02, 0.6, 30)])

    best = (0, 0.0, -1.0)
    for d in delays:
        for tau in taus:
            acc = rule_accuracy(theta, thetadot, u, int(d), tau)
            if not np.isnan(acc) and acc > best[2]:
                best = (int(d), float(tau), float(acc))

    acc_naive = rule_accuracy(theta, thetadot, u, 0, 0.0)   # the circular fit
    return best[0] * DT, best[1], best[2], acc_naive


def analyze(episodes):
    theta = np.concatenate([e.theta for e in episodes])
    thetadot = np.concatenate([e.thetadot for e in episodes])
    u = np.concatenate([e.u for e in episodes])

    print("\n" + "=" * 70)
    print(" What were you actually doing?")
    print("=" * 70)
    print(f"  {len(u)} samples over {len(episodes)} attempt(s), "
          f"best {max(e.t_end for e in episodes):.2f} s\n")

    right, left, none = u > 0, u < 0, u == 0

    print("  When you pressed        mean theta        mean thetadot")
    print("  " + "-" * 56)
    for label, mask in (("RIGHT (F > 0)", right),
                        ("nothing", none),
                        ("LEFT  (F < 0)", left)):
        if mask.any():
            print(f"  {label:20s} {np.rad2deg(theta[mask].mean()):+8.2f} deg"
                  f"    {np.rad2deg(thetadot[mask].mean()):+8.2f} deg/s")

    if not (right.any() and left.any()):
        print("\n  You only ever pressed one key. Play a little longer.")
        return

    # 1. The sign of the feedback.
    if theta[right].mean() > theta[left].mean():
        print("\n  You pushed RIGHT when the pole leaned RIGHT (theta > 0).")
        print("  That is not a mistake. Pushing the cart right rotates the pole")
        print("  left -- recall B[3] < 0 from Lab 1. You move the cart underneath")
        print("  the pole, the way you chase a broomstick on your palm.")
    else:
        print("\n  You pushed against the lean. With B[3] < 0 that drives the pole")
        print("  over faster, which is presumably why the attempts were short.")

    print("\n  Your input was never a number you chose in advance. It was a SIGN,")
    print("  and the sign came from how far the pole was from where you wanted it:")
    print("\n      error   e = theta - theta_desired = theta        (upright: theta_desired = 0)")
    print("\n  That is feedback. It is the whole reason you could do what no")
    print("  open-loop input in Lab 2 could.")

    # 2. Which past state explains the present key?
    delay, tau, acc, acc_naive = fit_policy(theta, thetadot, u)

    print("\n" + "=" * 70)
    print(" Bonus: reconstructing your control law")
    print("=" * 70)
    print("  A first guess would be to check the key against the error right now:")
    print(f"\n      u = F * sign(e)              reproduces {acc_naive*100:5.1f}% of your keys")
    print("\n  But that number is not trustworthy. This is a closed loop: the pole")
    print("  leans the way it leans partly BECAUSE of the key you are holding. The")
    print("  honest question is which PAST state explains the present key -- you")
    print("  cannot react to something you have not seen yet.")
    print("\n  Sweeping over reaction delay d and derivative ratio tau:\n")
    print(f"      u = F * sign( e(t-d) + tau * edot(t-d) )")
    print(f"\n      d   = {delay*1000:5.0f} ms      your reaction delay")
    print(f"      tau = {tau:5.3f} s       how far ahead you were looking")
    print(f"      -> reproduces {acc*100:5.1f}% of your keys")

    if tau > 0.02 and acc > acc_naive:
        print("\n  tau > 0, so you were not reacting to the error alone. You were also")
        print("  reacting to how fast it was growing -- you led the pole instead of")
        print("  chasing it. With a reaction delay of about a fifth of a second and")
        print("  an instability whose time constant is 0.25 s, chasing is not enough.")
        print("\n      u = kp * e  +  kd * edot           kd/kp = tau")
        print("\n  The e term is proportional feedback: how wrong am I now.")
        print("  The edot term is derivative feedback: where is the error headed.")
        print("  You have been running a PD controller with your fingers.")
    else:
        print("\n  The fit did not find a clear derivative term. Play a longer round")
        print("  (say 20+ seconds of survival) and the sweep will have more to work with.")

    print("\n  Nothing here was chosen ahead of time. You measured an error and")
    print("  reacted to it. That loop is the subject of the next chapter -- all that")
    print("  is left is to choose kp and kd on purpose instead of by instinct.")

    plot_phase_plane(theta, thetadot, u, delay, tau)


def plot_phase_plane(theta, thetadot, u, delay, tau):
    """Phase portrait of the operator's policy, with the fitted switching line.

    Points are plotted at the state the operator SAW (delayed), against the key
    they pressed after seeing it. Plot it against the current state instead and
    the switching line disappears into the closed-loop confound.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d = int(round(delay / DT))
    if d > 0:
        th, thd, key = theta[:-d], thetadot[:-d], u[d:]
    else:
        th, thd, key = theta, thetadot, u

    fig, ax = plt.subplots(1, 2, figsize=(12, 5))

    th_d, thd_d = np.rad2deg(th), np.rad2deg(thd)
    for mask, color, label in ((key > 0, "tab:red", "then pressed RIGHT (F > 0)"),
                               (key < 0, "tab:blue", "then pressed LEFT  (F < 0)"),
                               (key == 0, "0.8", "no key")):
        if mask.any():
            ax[0].scatter(th_d[mask], thd_d[mask], s=5, alpha=0.35,
                          c=color, label=label)

    # switching line:  e + tau*edot = 0   ->   edot = -e/tau
    if abs(tau) > 1e-6:
        lim = np.abs(th_d).max() * 1.05
        line = np.linspace(-lim, lim, 20)
        ax[0].plot(line, -line / tau, "k", lw=2,
                   label=f"e + {tau:.3f}*edot = 0")
        ax[0].set_ylim(np.percentile(thd_d, 0.5), np.percentile(thd_d, 99.5))

    ax[0].axhline(0, color="k", lw=0.4)
    ax[0].axvline(0, color="k", lw=0.4)
    ax[0].set_xlabel("error  e = theta [deg]   (as seen %d ms earlier)" % (delay * 1000))
    ax[0].set_ylabel("edot = thetadot [deg/s]")
    ax[0].set_title("Your policy in the phase plane")
    ax[0].legend(loc="upper right", fontsize=8)

    # The same story in one dimension. Collapse the phase plane onto the single
    # number the switching line depends on, and ask: given that number, how often
    # did you press RIGHT? A clean step through zero means the sign of that one
    # number is what decided the key.
    m = key != 0
    sigma = np.rad2deg(th[m] + tau * thd[m])
    is_right = (key[m] > 0).astype(float)

    edges = np.quantile(sigma, np.linspace(0, 1, 26))
    edges = np.unique(edges)
    idx = np.clip(np.digitize(sigma, edges[1:-1]), 0, len(edges) - 2)
    centers = 0.5 * (edges[:-1] + edges[1:])
    frac = np.array([is_right[idx == b].mean() if (idx == b).any() else np.nan
                     for b in range(len(centers))])

    ax[1].plot(centers, frac, "o-", color="tab:purple", ms=4)
    ax[1].axvline(0, color="k", lw=1)
    ax[1].axhline(0.5, color="k", lw=0.4, ls="--")
    ax[1].set_ylim(-0.05, 1.05)
    ax[1].set_xlabel(f"decision variable   e + {tau:.3f}*edot   [deg]")
    ax[1].set_ylabel("fraction of the time you pressed RIGHT")
    ax[1].set_title("The sign of that one number chose the key")

    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()

    RESULTS.mkdir(exist_ok=True)
    out = RESULTS / "03_human_policy.png"
    fig.savefig(out, dpi=120)
    print(f"\n  [saved] {out}")
    plt.close(fig)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--slow", type=float, default=3.0,
                    help="wall-clock slowdown factor (3 = one third speed)")
    ap.add_argument("--force", type=float, default=5.0,
                    help="force applied while an arrow key is held [N]")
    ap.add_argument("--auto", action="store_true",
                    help="let a delayed bang-bang operator play instead of you")
    ap.add_argument("--watch", action="store_true",
                    help="show the GUI during --auto (otherwise it runs headless)")
    args = ap.parse_args()

    episodes = run(args)
    if episodes and sum(len(e.u) for e in episodes) > 50:
        analyze(episodes)
    else:
        print("\n  Not enough data to analyze. Try again and play for a few seconds.")
