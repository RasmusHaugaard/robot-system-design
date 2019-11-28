import signal
import time
import os

print(os.getpid())


def stop_handler(signum, frame):
    print('Signal handler called with signal',
          signum)
    raise OSError("stop received!")


def long_function():
    time.sleep(100)
    print("SOMETHING AFTER")


signal.signal(signal.SIGUSR1, stop_handler)

try:
    long_function()
except OSError as e:
    print(e)
    raise
