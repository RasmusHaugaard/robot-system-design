import rtde_control
from pymodbus.client.sync import ModbusTcpClient

from rsd import conf
from rsd.mes import Mes
from rsd.robot import q


def main():
    mes = Mes(conf.MES_SERVER_URL)
    ur_ctrl = rtde_control.RTDEControlInterface(conf.UR_IP)
    ur_modbus_client = ModbusTcpClient(conf.UR_IP)
    pi_modbus_client = ModbusTcpClient(conf.PI_IP, conf.PI_MODBUS_PORT)
    move = ur_ctrl.moveJ

    def grip(do_grip=True):
        ur_modbus_client.write_coil(16 + conf.IO_GRIPPER, do_grip)

    def release():
        grip(False)

    def get_vision_color():
        values = pi_modbus_client.read_input_registers(0, 1)
        return values.registers[0]

    # empty potential load
    move(q.IDLE)
    move(q.BRICK_DROP_DISCARD_BOX)
    release()
    move(q.IDLE)

    orders_ready = False

    while True:

        # TODO: Request mir and wait for it to come.

        if orders_ready:
            # TODO: move the finished boxes onto the mir
            orders_ready = False

        # TODO: Move empty boxes from MIR to prod. table

        # TODO: Let the mir know, we're done

        # push-align empty order boxes
        move(q.ABOVE_PUSH_START_BOX_TRAY)
        move(q.PUSH_START_BOX_TRAY)
        move(q.PUSH_END_BOX_TRAY)
        move(q.ABOVE_PUSH_END_BOX_TRAY)
        move(q.IDLE)

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
                    move(q.IDLE)
                    move(q_above_brick)
                    move(q_grasp_brick)
                    grip()
                    move(q_above_brick)

                    move(q.ABOVE_CAMERA)
                    if get_vision_color() == brick_color_id:
                        ur_ctrl.moveJ(q_drop_order_box)
                        count -= 1
                    else:
                        ur_ctrl.moveJ(q.BRICK_DROP_DISCARD_BOX)
                    release()

            mes.complete_order(order, ticket)

        orders_ready = True
        move(q.IDLE)


if __name__ == '__main__':
    main()
