import redis
import pickle
import threading


class RsdRedis:
    def __init__(self, parser=pickle):
        self.r = redis.Redis()
        self.p = parser
        self.sub_threads = {}
        self.sub_i = 0
        self.pubsub = {}

    def publish(self, channel, data):
        self.r.publish(channel, self.p.dumps(data))

    def get(self, key):
        val = self.r.get(key)
        if val is not None:
            val = self.p.loads(val)
        return val

    def set(self, key, val):
        self.r.set(key, self.p.dumps(val))

    def subscribe(self, channel, cb, sync=True, blocking=False):
        sub_i = self.sub_i
        self.sub_i += 1

        p = self.r.pubsub()
        self.pubsub[sub_i] = p
        p.subscribe(channel)

        def sub():
            for msg in p.listen():
                if msg['type'] != 'message':
                    continue
                data = self.p.loads(msg['data'])
                if sync:
                    cb(data)
                else:
                    threading.Thread(target=lambda: cb(data)).start()

        t = threading.Thread(target=sub)
        t.start()
        self.sub_threads[sub_i] = t

        if blocking:
            t.join()

        return sub_i

    def unsubscribe(self, sub_i=None):
        if sub_i is not None:
            p = self.pubsub[sub_i]  # type: redis.client.PubSub
            p.unsubscribe()
            del self.pubsub[sub_i]
        else:
            for i in list(self.pubsub.keys()):
                self.unsubscribe(i)

    def join(self):
        for t in self.sub_threads.values():
            t.join()


def main():
    import time
    r = RsdRedis()
    r.subscribe("hello", lambda data: print("hello", data))
    r.publish("hello", "world")

    time.sleep(0.1)
    r.unsubscribe()
    print("test finished")


if __name__ == '__main__':
    main()
