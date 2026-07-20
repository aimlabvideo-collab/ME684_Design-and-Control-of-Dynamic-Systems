"""Load the arm and let gravity swing it. Ctrl-C to stop."""

import pybullet as p
import time

p.connect(p.GUI)                      # open 3-D viewer
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
