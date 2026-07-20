"""
ME 684 - Chapter 2: put the cart-pole on the screen

Loads assets/cartpole.urdf and lets it go. Nobody pushes the cart, so the
pole tips over on its own.

    python test.py

Ctrl-C in the terminal to stop.
"""

import time
import pybullet as p
import pybullet_data

DT = 1 / 240
BASE_Z = 1.2            # lift the rail so the pole never reaches the floor

CART_JOINT = 0          # joint indices inside the URDF
POLE_JOINT = 1

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())   # to find plane.urdf
p.setGravity(0, 0, -9.81)
p.setTimeStep(DT)

p.loadURDF("plane.urdf")
robot = p.loadURDF("assets/cartpole.urdf", [0, 0, BASE_Z], useFixedBase=True)

p.resetDebugVisualizerCamera(cameraDistance=4.0, cameraYaw=0,
                             cameraPitch=-15,
                             cameraTargetPosition=[0, 0, BASE_Z])

# GOTCHA 1: PyBullet adds its own damping to every body (angularDamping =
# 0.04 by default). Our equations have no friction term at all, so unless
# we switch it off the simulator is solving a different problem and
# model_test.py will never agree.
for link in range(-1, p.getNumJoints(robot)):
    p.changeDynamics(robot, link, linearDamping=0, angularDamping=0,
                     jointDamping=0)

# GOTCHA 2: straight after loadURDF every joint has a velocity motor
# switched on, holding it at zero speed with a large maximum force. That
# motor silently cancels any force you apply, and it also stops the pole
# from tipping. Setting its force to zero releases the joint.
p.setJointMotorControl2(robot, CART_JOINT, p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(robot, POLE_JOINT, p.VELOCITY_CONTROL, force=0)

# Start the pole 2 degrees off vertical. At exactly 0 it would balance
# forever: upright is an equilibrium, just an unstable one.
p.resetJointState(robot, POLE_JOINT, 0.035)

while True:
    p.stepSimulation()
    time.sleep(DT)

    x, xdot, _, _ = p.getJointState(robot, CART_JOINT)
    th, thdot, _, _ = p.getJointState(robot, POLE_JOINT)
    print(f"x = {x:+.3f} m   theta = {th:+.3f} rad")
