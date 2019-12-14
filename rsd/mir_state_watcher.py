#!/usr/bin/env python3

import rsd.mir.rest_api_functions as rest
import rsd.mir.mir as mir
from rsd.utils.redis_helper import RedisHelper
import time

r = RedisHelper()

while True:
    state = rest.get_register_value(mir.MIR_STATE_REGISTER)
    r.set("mir_state", mir.mir_states[state])
    time.sleep(2)