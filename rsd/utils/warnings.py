from rsd.utils.rsd_redis import RsdRedis


class Warnings:
    def __init__(self, r: RsdRedis):
        self.r = r

    def get_all_warnings(self):
        warnings = {}
        for warning_key in self.r.r.smembers("warnings"):
            warn_type = warning_key.decode("utf-8")[8:]
            warnings[warn_type] = self.r.get(warning_key)
        return warnings

    def set(self, type: str, text):
        key = "warning:{}".format(type)
        self.r.r.sadd("warnings", key)
        self.r.set(key, text)

    def rem(self, type):
        key = "warning:{}".format(type)
        self.r.r.srem("warnings", key)
