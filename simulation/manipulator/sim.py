"""Load the arm and let gravity swing it. Ctrl-C to stop."""

import pybullet as p
import time

p.connect(p.GUI)                      # open 3-D viewer

# The default camera sits 5 m away, aimed at the floor and turned 50 deg
# off to one side, so a 0.5 m arm shows up as a small slanted sliver.
# joint1 turns about the y axis, so the arm swings in the x-z plane. Put
# the camera on the y axis and it looks straight at that plane.
p.resetDebugVisualizerCamera(
    cameraDistance=2,                 # zoom -- smaller is closer
    cameraYaw=180,                    # on the +y side, looking back along -y
    cameraPitch=0,                    # level with the arm
    cameraTargetPosition=[0, 0, 0.3]) # aim at the middle of the arm, not the base

# Keep PyBullet's overlay on: this one flag draws the side panel, the camera
# preview panes, AND the red/green/blue world axes at the origin. Pass 0 for
# a clean window, but the axes disappear with everything else.
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

p.setGravity(0, 0, -9.81)             # turn on gravity

robot = p.loadURDF("one_link_arm.urdf", useFixedBase=True)

p.resetJointState(robot, 0, 0.3)      # start 0.3 rad off vertical

# loadURDF leaves a motor on at every joint, holding it still with up to
# 10 N*m (the effort in the URDF). Gravity only pulls with 2.45 N*m, so
# without this line the motor wins and the arm hangs there, motionless.
p.setJointMotorControl2(robot, 0, p.VELOCITY_CONTROL, force=0)

while True:                           # simulation loop
    p.stepSimulation()                # advance 1/240 s
    time.sleep(1 / 240)

    angle, rate, _, _ = p.getJointState(robot, 0)
    print(f"angle = {angle:+.3f} rad   rate = {rate:+.3f} rad/s")
