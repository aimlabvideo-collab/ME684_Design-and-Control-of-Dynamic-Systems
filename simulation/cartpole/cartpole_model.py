"""
ME 684 - Chapter 2: the model WE derived

This file is our side of the argument. It contains the physical constants,
the equations of motion we worked out on the board, and a way to step them
forward in time.

    It does not import PyBullet. It has never seen the simulator.

That is the whole point of Lab 1: if these equations and the simulator agree,
it cannot be because one copied the other.

The robot we compare against lives in cartpole_robot.py.

Coordinates                                        (put this on the board first)

    x       cart position [m], +x to the right
    theta   pole angle from straight up [rad], +theta leans right
    F       force on the cart [N], +F pushes right

    state   s = [x, theta, xdot, thetadot]
            s = 0 is upright -- and that equilibrium is UNSTABLE

The derivation of the equations below is Section 2.3 of the notes.
"""

import numpy as np


# --- physical constants ------------------------------------------------
# These MUST match assets/cartpole.urdf. If they drift apart, Lab 1b stops
# agreeing and it looks like your algebra is wrong when it is not.

G = 9.81            # gravity                       [m/s^2]
M_CART = 1.0        # cart mass          M          [kg]
M_POLE = 0.1        # pole mass          m          [kg]
L_POLE = 1.0        # pole total length  L          [m]
W_POLE = 0.05       # pole cross-section width      [m]
LC = L_POLE / 2.0   # hinge -> pole centre of mass  [m]

# Moment of inertia of a uniform box about its own centre of mass
I_POLE = M_POLE * (L_POLE**2 + W_POLE**2) / 12.0   # [kg m^2]


# --- the equations of motion -------------------------------------------
#
#   [ M+m           m lc cos th ] [ xddot  ]   [ F + m lc sin th thdot^2 ]
#   [ m lc cos th   I + m lc^2  ] [ thddot ] = [ m g lc sin th           ]
#
# This is the same matrix you will see written out by hand in 01a and 01b.
# It appears twice on purpose: there, so you can read the algebra at the
# moment it is being tested; here, so the labs that run hundreds of steps
# do not have to repeat it.

def nonlinear_dynamics(s, F):
    """State s and force F in, rate of change of the state out.

    Returns sdot = [xdot, thetadot, xddot, thddot]. No approximations.
    """
    _, th, xd, thd = s
    M, m, lc, I = M_CART, M_POLE, LC, I_POLE

    mass = np.array([[M + m, m * lc * np.cos(th)],
                     [m * lc * np.cos(th), I + m * lc**2]])
    rhs = np.array([F + m * lc * np.sin(th) * thd**2,
                    m * G * lc * np.sin(th)])

    xdd, thdd = np.linalg.solve(mass, rhs)      # the two accelerations

    # The first two entries are free: the rate of change of position IS
    # velocity. Only the last two needed the physics.
    return np.array([xd, thd, xdd, thdd])


# --- stepping it forward in time ---------------------------------------

def rk4_step(s, F, dt, dyn=nonlinear_dynamics):
    """Advance the state by one time step of dt seconds.

    nonlinear_dynamics tells us the rate of change right now. To get from
    'now' to 'dt from now' we have to integrate, and taking one straight
    step with the rate at the start (Euler) is crude. Runge-Kutta samples
    the rate four times across the interval and takes a weighted average,
    which is far more accurate for the same dt.

    https://en.wikipedia.org/wiki/Runge-Kutta_methods
    """
    k1 = dyn(s, F)                       # rate at the start
    k2 = dyn(s + 0.5 * dt * k1, F)       # rate at the midpoint, using k1
    k3 = dyn(s + 0.5 * dt * k2, F)       # midpoint again, using k2
    k4 = dyn(s + dt * k3, F)             # rate at the end
    return s + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
