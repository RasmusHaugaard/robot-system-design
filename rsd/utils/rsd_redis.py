from rsd.utils.redis_helper import RedisHelper


class RsdRedis(RedisHelper):

    def set_standard_digital_out(self, io_port, val):
        return self.service_call("setStandardDigitalOut", (io_port, val))

    def set_speed(self, val):
        return self.service_call("setSpeedDial", val)

    def get_q(self):
        return self.service_call("getActualQ", None)
