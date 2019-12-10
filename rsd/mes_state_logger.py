from rsd.mes import Mes
from rsd.utils.rsd_redis import RsdRedis
from rsd import conf

r = RsdRedis()
mes = Mes(conf.MES_SERVER_URL)


def on_state_changed(data):
    old_state, new_state = data
    mes.log_state(new_state)


r.subscribe("state_changed", on_state_changed)
r.join()
