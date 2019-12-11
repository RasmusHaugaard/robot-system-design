from rsd.conf import UR_IP
from rsd.utils.warnings import Warnings
from rsd.utils.redis_helper import RedisHelper

import os
import time

r = RedisHelper()
w = Warnings(r)
wt = "pi"

while True:
    if os.system('ping -c 1 -W 1 ' + UR_IP +  " > /dev/null 2>&1"): #Send one ping and wait max 1 second for response
        w.set(wt, "Connection to Raspberry Pi lost")
    else:   
        w.rem(wt)

    time.sleep(1)
