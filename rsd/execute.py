#!/usr/bin/python3

from rsd.robot.robot import Robot
from rsd.utils.rsd_redis import RsdRedis
from rsd import conf
from rsd.mes import Mes
from rsd.robot import q
from rsd.vision.vision_client import VisionClient
from rsd.robot.path import find_nearest, find_path
from rsd.packml.packml import PackMLActions as A
from rsd.mir.mir import Mir

redis = RsdRedis()
robot = Robot(redis)
mes = Mes(conf.MES_SERVER_URL)
vis = VisionClient()
mir = Mir(9, 11, "G9_mission")

# We don't know the position of the robot.
# Find the closest q and move a clear path from there to IDLE.
robot.release()
q_s, dist = find_nearest(robot.get_q())
clear_path = find_path(q_s, q.IDLE)
assert clear_path
robot.move(*clear_path)

orders_ready = False

while True:

    mir.come_to_workcell()

    if orders_ready:

        # move done orders onto the MIR
        for TRAY_GRASP, TRAY_GRASP_ABOVE, MIR_RELEASE in [
            (q.TRAY_GRASP_A, q.TRAY_GRASP_A_ABOVE, q.MIR_RELEASE_A),
            (q.TRAY_GRASP_B, q.TRAY_GRASP_B_ABOVE, q.MIR_RELEASE_B),
        ]:
            robot.move(
                TRAY_GRASP_ABOVE,
                TRAY_GRASP,
            ).grasp().move(
                TRAY_GRASP_ABOVE,
                q.MIR_WAYPOINT,
                q.MIR_RELEASE_WAYPOINT,
                MIR_RELEASE,
            ).release().move(
                q.MIR_RELEASE_WAYPOINT,
                q.MIR_WAYPOINT,
            )

        orders_ready = False

    # Move empty boxes from MIR to tray
    # align mir boxes
    robot.release().move(
        q.MIR_WAYPOINT,
        q.MIR_PUSH_START_ABOVE,
        q.MIR_PUSH_START,
        q.MIR_PUSH_END,
        q.MIR_PUSH_END_ABOVE,
    )
    for MIR_GRASP, MIR_GRASP_ABOVE, TRAY_RELEASE, TRAY_RELEASE_ABOVE in [
        (q.MIR_GRASP_A, q.MIR_GRASP_A_ABOVE, q.TRAY_RELEASE_A, q.TRAY_RELEASE_A_ABOVE),
        (q.MIR_GRASP_B, q.MIR_GRASP_B_ABOVE, q.TRAY_RELEASE_B, q.TRAY_RELEASE_B_ABOVE),
    ]:
        robot.move(
            MIR_GRASP_ABOVE,
            MIR_GRASP,
        ).grasp().move(
            MIR_GRASP_ABOVE,
            q.MIR_WAYPOINT,
            TRAY_RELEASE_ABOVE,
            TRAY_RELEASE,
        ).release().move(
            TRAY_RELEASE_ABOVE,
            q.MIR_WAYPOINT,
        )

    mir.release_from_workcell()

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
            failed = 0
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
                    failed = 0
                else:
                    robot.move(q.BRICK_DROP_DISCARD_BOX)
                    failed += 1
                    if failed >= 3:
                        redis.publish("action", A.HOLD)
                        failed = 0
                robot.release()

        mes.complete_order(order, ticket)

    orders_ready = True
    robot.move(q.IDLE)
