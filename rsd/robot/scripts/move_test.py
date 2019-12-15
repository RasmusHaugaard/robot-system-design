import rtde_control
import rtde_io
from rsd import conf
from rsd.robot.robot import Robot
from rsd.robot import q
import time

r = Robot()
r.r.set_speed(1)
r.move(
    q.IDLE, q.BRICK_DROP_DISCARD_BOX, q.IDLE
)