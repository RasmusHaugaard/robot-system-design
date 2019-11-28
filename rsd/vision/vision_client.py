from pymodbus.client.sync import ModbusTcpClient
from rsd import conf


class VisionClient:
    def __init__(self):
        self.modbus_conn = ModbusTcpClient(conf.PI_IP, conf.PI_MODBUS_PORT)

    def get_color_id(self):
        values = self.modbus_conn.read_input_registers(0, 1)
        return values.registers[0]
