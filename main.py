import rtde_control
from pymodbus.client.sync import ModbusTcpClient

from mes import Mes
import rsd_conf
import q


def main():
    mes = Mes(rsd_conf.MES_SERVER_URL)
    ur_ctrl = rtde_control.RTDEControlInterface(rsd_conf.UR_IP)
    ur_modbus_client = ModbusTcpClient(rsd_conf.UR_IP)
    pi_modbus_client = ModbusTcpClient(rsd_conf.PI_IP, rsd_conf.PI_MODBUS_PORT)
    move = ur_ctrl.moveJ

    def grip(do_grip=True):
        ur_modbus_client.write_coil(16 + rsd_conf.IO_GRIPPER, do_grip)

    def release():
        grip(False)

    def get_vision_color():
        values = pi_modbus_client.read_input_registers(0, 1)
        return values.registers[0]

    move(q.IDLE)
    release()

    while True:
        order, ticket = mes.take_a_ready_order()
        print(order)

        # pack the order in a box
        for brick_color_id in range(len(rsd_conf.BRICK_COLORS)):
            brick_color = rsd_conf.BRICK_COLORS[brick_color_id]
            q_grasp_brick, q_above_brick = q.GRASP_BRICKS[brick_color_id], q.ABOVE_BRICKS[brick_color_id]

            count = order[brick_color]
            while count > 0:
                move(q_above_brick)
                move(q_grasp_brick)
                grip()
                move(q_above_brick)

                move(q.ABOVE_CAMERA)
                if get_vision_color() == brick_color_id:
                    ur_ctrl.moveJ(q.BRICK_DROP_ORDER_BOX)
                    count -= 1
                else:
                    ur_ctrl.moveJ(q.BRICK_DROP_DISCARD_BOX)
                release()

        # TODO: Request mir and wait for it to come.

        # move the box onto the mir
        move(q.ABOVE_ORDER_BOX_EDGE)
        move(q.GRASP_ORDER_BOX_EDGE)
        grip()
        move(q.ABOVE_ORDER_BOX_EDGE_WITH_BOX)
        move(q.ORDER_BOX_DROP_MIR_WAYPOINT)
        move(q.ORDER_BOX_DROP_MIR)
        release()
        move(q.IDLE)

        # TODO: Let the mir know, we're done

        mes.complete_order(order, ticket)


if __name__ == '__main__':
    main()
