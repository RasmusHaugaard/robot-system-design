from pymodbus.server.sync import StartTcpServer
from pymodbus.datastore import ModbusSparseDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext


class SyncCallbackDataBlock(ModbusSparseDataBlock):
    def __init__(self, cb_map):
        self.cb_map = cb_map
        m = {}
        for key, cb in cb_map.items():
            m[key] = 0
        super().__init__(m)


    def getValues(self, address, count=1):
        return [self.cb_map[i]() for i in range(address, address + count)]


def start_cb_register_modbus_server(cb_map, port=5020):
    block = SyncCallbackDataBlock(cb_map)
    slaves = ModbusSlaveContext(di=None, co=None, hr=None, ir=block, zero_mode=True)
    context = ModbusServerContext(slaves=slaves, single=True)
    StartTcpServer(context, address=("0.0.0.0", port))


if __name__ == "__main__":
    i = 0


    def counter():
        global i
        i += 1
        return i


    start_cb_register_modbus_server({0x00: counter})
