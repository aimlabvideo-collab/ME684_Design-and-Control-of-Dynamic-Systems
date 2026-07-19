"""
ME 684 - Chapter 2: the robot we compare against

A thin wrapper around PyBullet, the physics engine. This is the 'real robot'
of Lab 1 -- it knows nothing about the equations in cartpole_model.py. You
hand it a description of the bodies and joints (assets/cartpole.urdf) and it
works out the motion itself.

Five things you can do to it:

    env = CartPole(gui=True, dt=...)   build it and open a window
    env.reset(s)                       teleport it to a state
    env.apply_force(F)                 push the cart
    env.step()                         run the physics for dt seconds
    env.get_state()                    read [x, theta, xdot, thetadot]

Two settings inside __init__ matter enormously and are easy to miss. Both
are marked GOTCHA below. Getting either wrong makes the simulator disagree
with correct algebra, and you will spend an afternoon blaming your algebra.
"""

import time
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data

ASSETS = Path(__file__).parent / "assets"

CART_JOINT = 0      # joint indices inside the URDF
POLE_JOINT = 1
BASE_Z = 1.2        # lift the rail so the pole never hits the floor


class CartPole:
    """The cart takes a force. The pole joint is left completely free.

    There is no motor on the pole -- one actuator, two degrees of freedom.
    That is what 'underactuated' means, and it is what makes balancing a
    control problem rather than bookkeeping.
    """

    def __init__(self, gui=True, dt=1.0 / 240.0, force_limit=100.0):
        self.dt = dt
        self.gui = gui
        self.force_limit = force_limit

        # connect to the engine: GUI opens a window, DIRECT runs headless
        self.cid = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(dt)

        if gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.resetDebugVisualizerCamera(
                cameraDistance=4.0, cameraYaw=0, cameraPitch=-15,
                cameraTargetPosition=[0, 0, BASE_Z],
            )

        # load the ground, then our robot
        p.loadURDF("plane.urdf")
        self.robot = p.loadURDF(
            str(ASSETS / "cartpole.urdf"), [0, 0, BASE_Z], useFixedBase=True
        )

        # GOTCHA 1 -- PyBullet adds its own damping to every body by default
        # (angularDamping = 0.04). Our equations have no friction term at
        # all, so unless we switch it off the simulator is solving a
        # different problem and Lab 1b will never agree.
        for link in range(-1, p.getNumJoints(self.robot)):
            p.changeDynamics(self.robot, link, linearDamping=0.0,
                             angularDamping=0.0, jointDamping=0.0)

        self._release_joints()

    def _release_joints(self):
        """GOTCHA 2 -- the most common PyBullet mistake there is.

        Straight after loadURDF, every joint has a velocity motor switched
        on, holding it at zero speed with a large maximum force. That motor
        silently cancels any torque you apply, so your control input looks
        like it does nothing whatsoever. Setting its force to zero releases
        the joint.
        """
        p.setJointMotorControl2(self.robot, CART_JOINT, p.VELOCITY_CONTROL,
                                force=0)
        p.setJointMotorControl2(self.robot, POLE_JOINT, p.VELOCITY_CONTROL,
                                force=0)

    def reset(self, s):
        """Teleport to s = [x, theta, xdot, thetadot]. No physics runs."""
        s = np.asarray(s, dtype=float)
        p.resetJointState(self.robot, CART_JOINT, s[0], s[2])
        p.resetJointState(self.robot, POLE_JOINT, s[1], s[3])
        self._release_joints()          # resetting re-enables the motors

    def get_state(self):
        """Read s = [x, theta, xdot, thetadot] out of the joints."""
        x, xd, _, _ = p.getJointState(self.robot, CART_JOINT)
        th, thd, _, _ = p.getJointState(self.robot, POLE_JOINT)
        return np.array([x, th, xd, thd])

    def apply_force(self, F):
        """Push the cart with F newtons. Returns F after the limit is applied.

        The force is cleared after every step(), so it has to be re-applied
        on every single one. For a sliding joint, TORQUE_CONTROL means a
        straight-line force.
        """
        F = float(np.clip(F, -self.force_limit, self.force_limit))
        p.setJointMotorControl2(self.robot, CART_JOINT, p.TORQUE_CONTROL,
                                force=F)
        return F

    def step(self):
        """Run the physics forward by exactly dt seconds."""
        p.stepSimulation()

    def close(self):
        p.disconnect(self.cid)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def simulate(env, input_fn, T, s0, realtime=None):
    """Run input_fn(t, s) -> F for T seconds. Returns (ts, S, U) histories.

    In Chapter 2 input_fn is not a controller yet -- it is just some signal
    we inject to see how the plant responds.
    """
    if realtime is None:
        realtime = env.gui

    env.reset(s0)
    n = int(round(T / env.dt))
    ts = np.zeros(n)
    S = np.zeros((n, 4))
    U = np.zeros(n)

    t_start = time.perf_counter()
    for k in range(n):
        t = k * env.dt
        s = env.get_state()
        u = env.apply_force(input_fn(t, s))
        env.step()

        ts[k], S[k], U[k] = t, s, u

        if realtime:
            # Not time.sleep(env.dt): Windows rounds a sleep up to about
            # 12 ms, which would run a small dt many times too slow. Wait
            # only for however long we are actually ahead of the clock.
            ahead = (k + 1) * env.dt - (time.perf_counter() - t_start)
            if ahead > 0:
                time.sleep(ahead)

    return ts, S, U


def plot_run(ts, S, U, title, save=None, show=True):
    """Cart, pole and input against time -- three stacked panels."""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)

    ax[0].plot(ts, S[:, 0], label="x [m]")
    ax[0].plot(ts, S[:, 2], "--", alpha=0.6, label="xdot [m/s]")
    ax[0].set_ylabel("cart")
    ax[0].legend(loc="upper right")

    ax[1].plot(ts, np.rad2deg(S[:, 1]), color="tab:red", label="theta [deg]")
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].set_ylabel("pole")
    ax[1].legend(loc="upper right")

    ax[2].plot(ts, U, color="tab:green", label="F [N]")
    ax[2].set_ylabel("input")
    ax[2].set_xlabel("time [s]")
    ax[2].legend(loc="upper right")

    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle(title)
    fig.tight_layout()

    if save:
        Path(save).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save, dpi=120)
        print(f"[saved] {save}")
    if show:
        plt.show()
    plt.close(fig)
