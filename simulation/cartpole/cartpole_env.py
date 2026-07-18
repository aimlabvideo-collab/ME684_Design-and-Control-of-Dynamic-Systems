"""
ME 684 - Chapter 2: Mathematical Modeling
Shared module. Every lab script in this chapter imports from here.

Contents:
    1. nonlinear_dynamics : the equations of motion we derive by hand
    2. linearized_model   : the (A, B) pair obtained by linearizing about upright
    3. CartPole           : a thin wrapper around the PyBullet simulator

--------------------------------------------------------------------------
1. Coordinates and sign conventions  (put this on the board first)
--------------------------------------------------------------------------
    x      : cart position [m],           +x is to the right
    theta  : pole angle from vertical-up [rad]
             theta > 0  <=>  pole leans toward +x (to the right)
    F      : force applied to the cart [N],  +F pushes the cart right

    state vector    s = [x, theta, xdot, thetadot]^T
    equilibrium     s = 0  (upright).  This equilibrium is UNSTABLE.

--------------------------------------------------------------------------
2. Equations of motion (Lagrange)
--------------------------------------------------------------------------
The pole is hinged on the cart; its center of mass (COM) sits a distance lc
from the hinge.

    cart position     : (x, 0)
    pole COM position : (x + lc*sin(th),  lc*cos(th))

Kinetic energy
    T = 1/2 * M * xdot^2
      + 1/2 * m * [ (xdot + lc*cos(th)*thdot)^2 + (lc*sin(th)*thdot)^2 ]
      + 1/2 * I * thdot^2

Potential energy
    V = m * g * lc * cos(th)

Applying  d/dt(dL/dqdot) - dL/dq = Q  with  q = [x, th]  gives

    (M + m) * xddot  +  m*lc*cos(th) * thddot  -  m*lc*sin(th) * thdot^2  =  F
    m*lc*cos(th) * xddot  +  (I + m*lc^2) * thddot  -  m*g*lc*sin(th)     =  0

In matrix form -- this is exactly what nonlinear_dynamics() implements:

    [ M+m           m*lc*cos(th) ] [ xddot  ]   [ F + m*lc*sin(th)*thdot^2 ]
    [ m*lc*cos(th)  I + m*lc^2   ] [ thddot ] = [ m*g*lc*sin(th)           ]

--------------------------------------------------------------------------
3. Linearization about upright (sin th ~ th, cos th ~ 1, thdot^2 ~ 0)
--------------------------------------------------------------------------
    Let  D = (I + m*lc^2)(M + m) - (m*lc)^2.   Then

        xddot  = (  (I + m*lc^2) * F  -  m^2*g*lc^2 * th      ) / D
        thddot = ( -m*lc * F          +  (M+m)*m*g*lc * th    ) / D

Two facts worth dwelling on:

  (a) A[3,1] = (M+m)*m*g*lc/D > 0.
      thddot grows with th and has the SAME sign, so th runs away.
      One eigenvalue of A sits at +3.97 rad/s, in the right half plane.
      The upright equilibrium is unstable: left alone, the pole always falls.

  (b) B[3] = -m*lc/D < 0.
      Pushing the cart right (F > 0) makes the pole rotate LEFT (thddot < 0).
      So to catch a pole that is falling right (th > 0), you push RIGHT --
      you move the cart underneath it, the same way you chase a broomstick
      you are balancing on your palm.
      Get this sign wrong and every controller gain in later chapters flips.

Whether this derivation is actually correct is not something you should take
on faith. 01_modeling.py checks it against the simulator directly.
"""

import time
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data

ASSETS = Path(__file__).parent / "assets"

# ------------------------------------------------------------ physical constants
# These MUST match assets/cartpole.urdf.
G = 9.81            # gravity                       [m/s^2]
M_CART = 1.0        # cart mass          M          [kg]
M_POLE = 0.1        # pole mass          m          [kg]
L_POLE = 1.0        # pole total length  L          [m]
W_POLE = 0.05       # pole cross-section width      [m]
LC = L_POLE / 2.0   # hinge -> pole COM distance    [m]

# Moment of inertia of a uniform box about its COM, y axis
I_POLE = M_POLE * (L_POLE**2 + W_POLE**2) / 12.0   # [kg m^2]

# Joint indices inside the URDF
CART_JOINT = 0
POLE_JOINT = 1

BASE_Z = 1.2        # lift the rail so the pole never clips through the floor

STATE_NAMES = ["x [m]", "theta [rad]", "xdot [m/s]", "thetadot [rad/s]"]


# ======================================================================
#  The model we derive by hand
# ======================================================================
def nonlinear_dynamics(s, F):
    """Equations of motion.  s = [x, th, xdot, thdot], input F [N]  ->  sdot.

    This is a direct transcription of the matrix equation in section 2 of the
    module docstring. No approximations anywhere.
    """
    _, th, xd, thd = s
    M, m, lc, I = M_CART, M_POLE, LC, I_POLE
    st, ct = np.sin(th), np.cos(th)

    Mmat = np.array([[M + m,        m * lc * ct],
                     [m * lc * ct,  I + m * lc**2]])
    rhs = np.array([F + m * lc * st * thd**2,
                    m * G * lc * st])

    xdd, thdd = np.linalg.solve(Mmat, rhs)
    return np.array([xd, thd, xdd, thdd])


def linear_dynamics(s, F):
    """Linearized model as a drop-in replacement for nonlinear_dynamics()."""
    A, B = linearized_model()
    return A @ s + B.flatten() * F


def linearized_model():
    """(A, B) linearized about the upright equilibrium:  sdot = A s + B u,  u = F."""
    M, m, lc, I = M_CART, M_POLE, LC, I_POLE
    D = (I + m * lc**2) * (M + m) - (m * lc) ** 2

    A = np.zeros((4, 4))
    A[0, 2] = 1.0
    A[1, 3] = 1.0
    A[2, 1] = -(m**2) * G * lc**2 / D
    A[3, 1] = (M + m) * m * G * lc / D      # > 0  =>  unstable

    B = np.zeros((4, 1))
    B[2, 0] = (I + m * lc**2) / D
    B[3, 0] = -m * lc / D                   # < 0  =>  push right, pole tips left
    return A, B


def rk4_step(s, F, dt, dyn=nonlinear_dynamics):
    """Advance dyn(s, F) -> sdot by one step of 4th-order Runge-Kutta.

    We need this because comparing our model against the simulator means
    integrating our model forward in time, not just evaluating accelerations.
    link : https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
    """
    k1 = dyn(s, F)
    k2 = dyn(s + 0.5 * dt * k1, F)
    k3 = dyn(s + 0.5 * dt * k2, F)
    k4 = dyn(s + dt * k3, F)
    return s + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def rollout_model(dyn, s0, U, dt):
    """Integrate dyn from s0 through the input sequence U. Returns the trajectory."""
    S = np.zeros((len(U), 4))
    s = np.array(s0, dtype=float)
    for k, u in enumerate(U):
        S[k] = s
        s = rk4_step(s, u, dt, dyn)
    return S


# ======================================================================
#  The simulator
# ======================================================================
class CartPole:
    """PyBullet cart-pole.

    The cart joint takes a force input. The pole joint is left completely free:
    there is no motor on the pole. That is what makes this system underactuated
    -- one actuator, two degrees of freedom.
    """

    def __init__(self, gui=True, dt=1.0 / 240.0, force_limit=100.0):
        self.dt = dt
        self.gui = gui
        self.force_limit = force_limit

        self.cid = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -G)
        p.setTimeStep(dt)

        if gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.resetDebugVisualizerCamera(
                cameraDistance=4.0, cameraYaw=0, cameraPitch=-15,
                cameraTargetPosition=[0, 0, BASE_Z],
            )

        p.loadURDF("plane.urdf")
        self.robot = p.loadURDF(
            str(ASSETS / "cartpole.urdf"), [0, 0, BASE_Z], useFixedBase=True
        )

        # PyBullet applies its own damping by default (angularDamping = 0.04).
        # Our hand-derived equations contain no such term, so turn it off or the
        # model and the simulator will disagree.
        for link in range(-1, p.getNumJoints(self.robot)):
            p.changeDynamics(self.robot, link, linearDamping=0.0,
                             angularDamping=0.0, jointDamping=0.0)

        self._setup_motors()

    def _setup_motors(self):
        """*** The single most common PyBullet mistake ***

        Right after loadURDF, every joint has a velocity motor enabled with
        targetVelocity = 0 and a large max force. If you do not disable it, that
        motor cancels whatever torque you apply, and it looks like your control
        input does nothing at all. Setting force = 0 releases the joint.
        """
        p.setJointMotorControl2(self.robot, CART_JOINT, p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(self.robot, POLE_JOINT, p.VELOCITY_CONTROL, force=0)

    def reset(self, s0):
        """Set the state to s0 = [x, theta, xdot, thetadot]."""
        s0 = np.asarray(s0, dtype=float)
        p.resetJointState(self.robot, CART_JOINT, s0[0], s0[2])
        p.resetJointState(self.robot, POLE_JOINT, s0[1], s0[3])
        self._setup_motors()

    def get_state(self):
        """Read s = [x, theta, xdot, thetadot]."""
        x, xd, _, _ = p.getJointState(self.robot, CART_JOINT)
        th, thd, _, _ = p.getJointState(self.robot, POLE_JOINT)
        return np.array([x, th, xd, thd])

    def apply_force(self, F):
        """Apply force F [N] to the cart. Returns the value after saturation.

        For a prismatic joint, TORQUE_CONTROL means a linear force.
        This force is cleared after every stepSimulation(), so it has to be
        re-applied on every single step.
        """
        F = float(np.clip(F, -self.force_limit, self.force_limit))
        p.setJointMotorControl2(self.robot, CART_JOINT, p.TORQUE_CONTROL, force=F)
        return F

    def step(self):
        p.stepSimulation()

    def close(self):
        p.disconnect(self.cid)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def simulate(env, input_fn, T, s0, realtime=None):
    """Run input_fn(t, s) -> F for T seconds. Returns (t, S, U) histories.

    In Chapter 2 input_fn is not a controller yet -- it is just some signal we
    inject to see how the plant responds.
    """
    if realtime is None:
        realtime = env.gui

    env.reset(s0)
    n = int(round(T / env.dt))
    ts = np.zeros(n)
    S = np.zeros((n, 4))
    U = np.zeros(n)

    for k in range(n):
        t = k * env.dt
        s = env.get_state()
        u = env.apply_force(input_fn(t, s))
        env.step()

        ts[k], S[k], U[k] = t, s, u
        if realtime:
            time.sleep(env.dt)

    return ts, S, U


# ======================================================================
#  Plotting helper
# ======================================================================
def plot_run(ts, S, U, title, save=None, show=True):
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
