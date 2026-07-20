"""Check that PyBullet is installed. An empty 3-D window should open."""

import pybullet as p

p.connect(p.GUI)
input("Press Enter to quit")
