from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from rsd import conf

client = ModbusClient(conf.PI_IP, conf.PI_MODBUS_PORT)
value = client.read_input_registers(0, 1)
print('Received value is: ', value.registers[0])
