from rtde_control import RTDEControlInterface
from pymodbus.client.sync import ModbusTcpClient
from rsd import conf
from rsd.robot import q

ur_ctrl = RTDEControlInterface(conf.UR_IP)
ur_modbus_client = ModbusTcpClient(conf.UR_IP)
move = ur_ctrl.moveJ


def grip(do_grip):
    ur_modbus_client.write_coil(16 + conf.IO_GRIPPER, do_grip)


color_id = conf.BRICK_COLORS.index("blue")
above_brick = q.ABOVE_BRICKS[color_id]
grasp_brick = q.GRASP_BRICKS[color_id]

move(q.IDLE)
grip(False)
move(above_brick)
move(grasp_brick)
grip(True)
move(above_brick)
move(q.ABOVE_CAMERA)
