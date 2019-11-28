#!/usr/bin/python3

from rsd.robot.robot import Robot
from rsd.utils.rsd_redis import RsdRedis
from rsd import conf
from rsd.mes import Mes
from rsd.robot import q
from rsd.vision.vision_client import VisionClient
from rsd.robot.path import find_nearest, find_path

redis = RsdRedis()
robot = Robot(redis)
mes = Mes(conf.MES_SERVER_URL)
vis = VisionClient()

# We don't know the position of the robot.
# Find the closest q and move a clear path from there to IDLE.
q_s, dist = find_nearest(robot.get_q())
clear_path = find_path(q_s, q.IDLE)
assert clear_path
robot.move(*clear_path)

# empty potential load
robot.move(q.BRICK_DROP_DISCARD_BOX)
robot.release()
robot.move(q.IDLE)

orders_ready = False

while True:
    # TODO: Request mir and wait for it to come.
    if orders_ready:
        # TODO: move the finished boxes onto the mir
        orders_ready = False
    # TODO: Move empty boxes from MIR to prod. table
    # TODO: Let the mir know, we're done

    # push-align empty order boxes
    robot.move(
        q.ABOVE_PUSH_START_BOX_TRAY,
        q.PUSH_START_BOX_TRAY,
        q.PUSH_END_BOX_TRAY,
        q.ABOVE_PUSH_END_BOX_TRAY,
        q.IDLE,
    )

    for box_id in range(4):
        q_drop_order_box = q.BRICK_DROP_ORDER_BOX[box_id]
        order, ticket = mes.take_a_ready_order()
        print(order)

        # pack the order in the box
        for brick_color_id in range(len(conf.BRICK_COLORS)):
            brick_color = conf.BRICK_COLORS[brick_color_id]
            q_grasp_brick, q_above_brick = q.GRASP_BRICKS[brick_color_id], q.ABOVE_BRICKS[brick_color_id]

            count = order[brick_color]
            while count > 0:
                robot.move(
                    q.IDLE,
                    q_above_brick,
                    q_grasp_brick,
                ).grasp().move(
                    q_above_brick,
                    q.ABOVE_CAMERA,
                )
                if vis.get_color_id() == brick_color_id:
                    robot.move(q_drop_order_box)
                    count -= 1
                else:
                    robot.move(q.BRICK_DROP_DISCARD_BOX)
                robot.release()

        mes.complete_order(order, ticket)

    orders_ready = True
    robot.move(q.IDLE)
