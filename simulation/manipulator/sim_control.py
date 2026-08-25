"""Drive the joint with a sine wave. Ctrl-C to stop."""

import pybullet as p
import math
import time

p.connect(p.GUI)

# Same camera as sim.py: on the y axis, looking straight at the x-z plane
# the arm swings in. See sim.py for what each number does.
p.resetDebugVisualizerCamera(
    cameraDistance=2,
    cameraYaw=180,
    cameraPitch=0,
    cameraTargetPosition=[0, 0, 0.3])
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

p.setGravity(0, 0, -9.81)

robot = p.loadURDF("one_link_arm.urdf", useFixedBase=True)

t = 0
while True:
    target = 0.5 * math.sin(t)        # where we want the joint to be

    p.setJointMotorControl2(
        robot, 0,                     # joint 0
        p.POSITION_CONTROL,
        targetPosition=target)

    p.stepSimulation()
    time.sleep(1 / 240)
    t += 1 / 240

    angle, _, _, _ = p.getJointState(robot, 0)
    print(f"target = {target:+.3f}   actual = {angle:+.3f} rad")

# The actual angle lags behind the target, and the lag grows if you speed
# the wave up. POSITION_CONTROL means PyBullet runs the motor controller
# for you -- from Chapter 3 on, we write that part ourselves.
