import threading


class FunctionQueue:
    def __init__(self):
        self.lock = threading.Lock()

    def __call__(self, fun, args=(), spread=None):
        if spread is None:
            spread = iter(args)
        self.lock.acquire()
        res = fun(*args if spread else args)
        self.lock.release()
        return res


def main():
    import time
    import random

    num_threads = 100

    class Inc:
        val = 0

        def __call__(self):
            time.sleep(random.random() / num_threads)
            temp = self.val
            time.sleep(random.random() / num_threads)
            self.val = temp + 1

    print("naive")
    inc = Inc()
    threads = []
    for i in range(num_threads):
        t = threading.Thread(target=inc)
        t.start()
        threads.append(t)
    for t in threads:
        t.join()
    print(inc.val, "should be", num_threads)

    print("\nwith function queue")
    inc = Inc()
    seq = FunctionQueue()
    threads = []
    for i in range(num_threads):
        t = threading.Thread(target=lambda: seq(inc))
        t.start()
        threads.append(t)
    for t in threads:
        t.join()
    print(inc.val, "should be", num_threads)

    print()
    if inc.val == num_threads:
        print("test succeeded")
    else:
        print("test failed")


if __name__ == '__main__':
    main()
