import rtde_control
from pymodbus.client.sync import ModbusTcpClient

import rsd_conf
import q


def main():
    ur_ctrl = rtde_control.RTDEControlInterface(rsd_conf.UR_IP)
    ur_ctrl.moveJ(q.IDLE)

    ur_modbus_client = ModbusTcpClient(rsd_conf.UR_IP)
    ur_modbus_client.write_coil(16 + rsd_conf.IO_GRIPPER, False)

    pi_modbus_client = ModbusTcpClient(rsd_conf.PI_IP, rsd_conf.PI_MODBUS_PORT)

    def gripper_grasp(grasp_q, pre_post_q):
        ur_ctrl.moveJ(pre_post_q)
        ur_ctrl.moveJ(grasp_q)
        ur_modbus_client.write_coil(16 + rsd_conf.IO_GRIPPER, True)
        ur_ctrl.moveJ(pre_post_q)

    def gripper_release():
        ur_modbus_client.write_coil(16 + rsd_conf.IO_GRIPPER, False)

    order = {"blue": 0, "red": 0, "yellow": 20}
    for brick_color_id in range(len(rsd_conf.BRICK_COLORS)):
        brick_color = rsd_conf.BRICK_COLORS[brick_color_id]
        q_grasp_brick, q_above_brick = q.GRASP_BRICKS[brick_color_id], q.ABOVE_BRICKS[brick_color_id]

        count = order[brick_color]
        for _ in range(count):
            gripper_grasp(q_grasp_brick, q_above_brick)
            ur_ctrl.moveJ(q.ABOVE_CAMERA)
            pi_modbus_client.read_input_registers(0, 1)
            ur_ctrl.moveJ(q.BRICK_DROP_ORDER_BOX)
            gripper_release()

    ur_ctrl.moveJ(q.IDLE)


if __name__ == '__main__':
    main()
