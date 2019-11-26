from pymodbus.client.sync import ModbusTcpClient as ModbusClient

# Replace with RPi address
client = ModbusClient('192.168.1.3',5020)
value=client.read_input_registers(0,1)
print('Received value is: ',value.registers[0])