"""
ME 684 - Chapter 3: meet the arm

Loads the one-link arm with gravity ON and lets it go. No controller, so it
falls and swings down to hang straight below the hinge.

    python test.py

This is the plant the PID slides design against. Held horizontal it needs
5 N*m just to fight gravity -- that 5 N*m is the disturbance the whole
chapter is about. Its numbers:

    J   = 0.5  kg m^2     inertia about the joint
    mgl = 5    N m        gravity torque when horizontal
    b   = 0.2  N m s/rad  joint friction (added in the control scripts)

Ctrl-C to stop.
"""

import time
import pybullet as p
import pybullet_data

DT = 1 / 240
JOINT = 0

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(DT)

p.loadURDF("plane.urdf", [0, 0, -1.0])

# URDF_USE_INERTIA_FROM_FILE: without it PyBullet ignores the <inertia> in
# the file and recomputes it from the box shape, which would NOT give our
# J = 0.5. The flag makes it honour the numbers we wrote.
robot = p.loadURDF("assets/one_link_arm.urdf", [0, 0, 0.5],
                   useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)

p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=90, cameraPitch=-10,
                             cameraTargetPosition=[0, 0, 0.5])

# Release the default velocity motor (see the note in step1_p.py); without
# this the joint is locked and the arm cannot fall.
p.setJointMotorControl2(robot, JOINT, p.VELOCITY_CONTROL, force=0)

# Start the arm horizontal so you can watch gravity pull it down.
p.resetJointState(robot, JOINT, 1.5708)

print("no controller -- gravity wins. Ctrl-C to stop.")
while True:
    p.stepSimulation()
    time.sleep(DT)
    angle, _, _, _ = p.getJointState(robot, JOINT)
    print(f"theta = {angle:+.3f} rad", end="\r")
