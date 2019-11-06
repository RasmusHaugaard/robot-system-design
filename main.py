import rtde_control
import rtde_io
from pymodbus.client.sync import ModbusTcpClient
import random

from mes import Mes
import rsd_conf
import q


def main():
    #mes = Mes(rsd_conf.MES_SERVER_URL)
    ur_ctrl = rtde_control.RTDEControlInterface(rsd_conf.UR_IP)
    #ur_io = rtde_io.RTDEIOInterface(rsd_conf.UR_IP)
    #ur_io.setStandardDigitalOut(rsd_conf.IO_GRIPPER, False)
    ur_ctrl.moveJ(q.IDLE)

    ur_modbus_client = ModbusTcpClient(rsd_conf.UR_IP)
    ur_modbus_client.write_coil(16 + rsd_conf.IO_GRIPPER, False)

    # pi_modbus_client = ModbusTcpClient(rsd_conf.PI_IP)

    def gripper_grasp(grasp_q, pre_post_q):
        ur_ctrl.moveJ(pre_post_q)
        ur_ctrl.moveJ(grasp_q)
        #ur_io.reconnect()
        #ur_io.setStandardDigitalOut(rsd_conf.IO_GRIPPER, True)
        ur_modbus_client.write_coil(16 + rsd_conf.IO_GRIPPER, True)
        ur_ctrl.moveJ(pre_post_q)

    def gripper_release():
        #ur_io.reconnect()
        #ur_io.setStandardDigitalOut(rsd_conf.IO_GRIPPER, False)
        ur_modbus_client.write_coil(16 + rsd_conf.IO_GRIPPER, False)

    while True:
        # order, ticket = mes.take_a_ready_order()
        order = {"blue": 1, "yellow": 1, "red": 1}

        # pack the order in a box
        for brick_id in range(3):
            brick_color = rsd_conf.BRICK_COLORS[brick_id]
            q_grasp_brick, q_above_brick = q.GRASP_BRICKS[brick_id], q.ABOVE_BRICKS[brick_id]

            count = order[brick_color]
            while count > 0:
                gripper_grasp(q_grasp_brick, q_above_brick)
                ur_ctrl.moveJ(q.ABOVE_CAMERA)
                brick_has_right_color = random.random() < 0.8  # pi_modbus_client.read_coils(brick_id)
                if brick_has_right_color:
                    ur_ctrl.moveJ(q.BRICK_DROP_ORDER_BOX)
                    count -= 1
                else:
                    ur_ctrl.moveJ(q.BRICK_DROP_DISCARD_BOX)
                gripper_release()

        # TODO: Request mir and wait for it to come.

        # move the box onto the mir
        gripper_grasp(q.GRASP_ORDER_BOX_EDGE, q.ABOVE_ORDER_BOX_EDGE)
        ur_ctrl.moveJ(q.ABOVE_ORDER_BOX_EDGE_WITH_BOX)
        ur_ctrl.moveJ(q.ORDER_BOX_DROP_MIR_WAYPOINT)
        ur_ctrl.moveJ(q.ORDER_BOX_DROP_MIR)
        gripper_release()
        ur_ctrl.moveJ(q.IDLE)

        # TODO: Let the mir know, we're done

        # mes.complete_order(order, ticket)


if __name__ == '__main__':
    main()
