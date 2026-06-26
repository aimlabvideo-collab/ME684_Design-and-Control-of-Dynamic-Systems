"""
ME 684 - Simulation Track Demo
Cart-Pole balancing with a state-feedback (LQR-style) controller in PyBullet.

This is the COMMON BENCHMARK used to compare simulation tools (PyBullet vs.
Webots vs. MATLAB/Simscape). The control-design idea is identical in every tool;
only the "plant" (physics engine) changes.

Pedagogical flow students follow:
  1. Write the linearized cart-pole model (Chapter 3, state-variable models).
  2. Place closed-loop poles / design K  (state feedback).
  3. Drop K onto the *nonlinear* physics plant here and see if it survives.
  4. Push the cart (disturbance) and watch it recover.

Run modes:
  python cartpole_pybullet.py            # headless, saves plot + gif
  python cartpole_pybullet.py --gui      # opens the PyBullet 3D window (in class)

Dependencies: pybullet, numpy, matplotlib, imageio
"""

import argparse
import numpy as np
import pybullet as p
import pybullet_data

# ----------------------------------------------------------------------
# 1. CONTROL DESIGN  (this is the part students actually do by hand)
# ----------------------------------------------------------------------
# Linearized cart-pole about the upright position.
# State x = [cart_pos, cart_vel, pole_angle, pole_rate]
# Parameters roughly match PyBullet's built-in cartpole.urdf
M = 1.0     # cart mass [kg]
m = 0.1     # pole mass [kg]
L = 0.5     # pole half-length [m]
g = 9.81

# Continuous-time A, B (standard inverted-pendulum-on-cart linearization)
A = np.array([
    [0, 1, 0, 0],
    [0, 0, -m * g / M, 0],
    [0, 0, 0, 1],
    [0, 0, (M + m) * g / (M * L), 0],
])
B = np.array([[0], [1 / M], [0], [-1 / (M * L)]])


def lqr(A, B, Q, R):
    """Solve continuous-time LQR via the algebraic Riccati equation.
    Uses scipy if available, otherwise a small iterative fallback so the
    script runs even on a bare Python install."""
    try:
        from scipy.linalg import solve_continuous_are
        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ P
        return K
    except Exception:
        # Iterative Riccati (kleinman) fallback
        P = Q.copy()
        for _ in range(10000):
            P_next = Q + A.T @ P + P @ A - P @ B @ np.linalg.inv(R) @ B.T @ P
            P = P + 1e-3 * P_next
            if np.linalg.norm(P_next) < 1e-9:
                break
        K = np.linalg.inv(R) @ B.T @ P
        return K


Q = np.diag([10.0, 1.0, 100.0, 1.0])   # penalize position & angle
R = np.array([[0.1]])
K = lqr(A, B, Q, R)
print("Designed gain K =", np.round(K, 2))


# ----------------------------------------------------------------------
# 2. PHYSICS PLANT  (PyBullet does the hard nonlinear integration for free)
# ----------------------------------------------------------------------
def run(gui=False, T=8.0, push_time=4.0, push_force=12.0):
    mode = p.GUI if gui else p.DIRECT
    cid = p.connect(mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -g)
    dt = 1.0 / 240.0
    p.setTimeStep(dt)

    robot = p.loadURDF("cartpole.urdf", [0, 0, 0])
    cart_joint, pole_joint = 0, 1

    # Disable default motors so we can apply our own force/torque
    for j in (cart_joint, pole_joint):
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, force=0)

    # Start the pole slightly off vertical so the controller has work to do
    p.resetJointState(robot, pole_joint, targetValue=0.15)

    steps = int(T / dt)
    log = {k: [] for k in ("t", "x", "xdot", "th", "thdot", "u")}
    frames = []
    cam_every = int(steps / 120)  # ~120 gif frames

    for i in range(steps):
        t = i * dt
        cart = p.getJointState(robot, cart_joint)
        pole = p.getJointState(robot, pole_joint)
        x, xdot = cart[0], cart[1]
        th, thdot = pole[0], pole[1]

        # ---- control law: u = -K x ----
        state = np.array([x, xdot, th, thdot])
        u = float(-K @ state)
        u = np.clip(u, -50, 50)
        p.setJointMotorControl2(robot, cart_joint, p.TORQUE_CONTROL, force=u)

        # ---- external disturbance: shove the cart once ----
        if abs(t - push_time) < dt:
            p.applyExternalForce(robot, cart_joint, [push_force, 0, 0],
                                 [0, 0, 0], p.LINK_FRAME)

        p.stepSimulation()

        for key, val in zip(("t", "x", "xdot", "th", "thdot", "u"),
                            (t, x, xdot, th, thdot, u)):
            log[key].append(val)

        if not gui and cam_every and i % cam_every == 0:
            w, h, rgb, _, _ = p.getCameraImage(
                320, 240,
                viewMatrix=p.computeViewMatrix([0, -3, 0.5], [0, 0, 0.3], [0, 0, 1]),
                projectionMatrix=p.computeProjectionMatrixFOV(60, 320 / 240, 0.1, 10),
            )
            frames.append(np.reshape(rgb, (h, w, 4))[:, :, :3].astype(np.uint8))

    p.disconnect()
    return log, frames


# ----------------------------------------------------------------------
# 3. OUTPUTS
# ----------------------------------------------------------------------
def save_outputs(log, frames, prefix="cartpole_pybullet"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.array(log["t"])
    fig, ax = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    ax[0].plot(t, np.degrees(log["th"]), color="#c0392b")
    ax[0].axhline(0, ls="--", c="gray", lw=0.8)
    ax[0].set_ylabel("pole angle [deg]")
    ax[0].set_title("Cart-Pole — LQR state feedback on PyBullet physics")
    ax[1].plot(t, log["x"], color="#2c6fbb")
    ax[1].set_ylabel("cart position [m]")
    ax[2].plot(t, log["u"], color="#1a1a1a")
    ax[2].set_ylabel("control force u [N]")
    ax[2].set_xlabel("time [s]")
    for a in ax:
        a.grid(alpha=0.3)
    # mark the disturbance
    for a in ax:
        a.axvspan(3.98, 4.05, color="orange", alpha=0.3)
    fig.tight_layout()
    fig.savefig(f"{prefix}_plot.png", dpi=130)
    print(f"saved {prefix}_plot.png")

    if frames:
        import imageio
        imageio.mimsave(f"{prefix}.gif", frames, fps=20)
        print(f"saved {prefix}.gif  ({len(frames)} frames)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", action="store_true", help="open 3D window")
    args = ap.parse_args()
    log, frames = run(gui=args.gui)
    final_angle = np.degrees(log["th"][-1])
    print(f"final pole angle = {final_angle:.2f} deg  (0 = balanced)")
    if not args.gui:
        save_outputs(log, frames)
