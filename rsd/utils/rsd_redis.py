import redis
import pickle
import threading


class RsdRedis:
    def __init__(self, parser=pickle):
        self.r = redis.Redis()
        self.p = parser
        self.threads = {}
        self.sub_i = 0
        self.pubsub = {}

    def get(self, key):
        val = self.r.get(key)
        if val is not None:
            val = self.p.loads(val)
        return val

    def set(self, key, val):
        self.r.set(key, self.p.dumps(val))

    def publish(self, channel, data):
        self.r.publish(channel, self.p.dumps(data))

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
        self.threads[sub_i] = t

        if blocking:
            self.join(sub_i)

        return sub_i

    def unsubscribe(self, sub_i=None):
        if sub_i is not None:
            p = self.pubsub[sub_i]  # type: redis.client.PubSub
            p.unsubscribe()
            del self.pubsub[sub_i]
        else:
            for i in list(self.pubsub.keys()):
                self.unsubscribe(i)

    def join(self, sub_i=None):
        if sub_i is None:
            for i in list(self.threads.keys()):
                self.join(i)
        else:
            self.threads[sub_i].join()
            del self.threads[sub_i]

    def service(self, name, cb, sync=True, blocking=False):
        def _cb(service_data):
            req_id, cb_data = service_data
            response = cb(cb_data)
            self.publish("res:{}".format(req_id), response)

        return self.subscribe("service:{}".format(name), _cb, sync, blocking)

    def service_call(self, name, args):
        req_id = self.r.incr("req:id")
        obj = {}

        def store_unsub(val):
            obj["val"] = val
            self.unsubscribe(sub)

        sub = self.subscribe("res:{}".format(req_id), store_unsub, blocking=False)
        self.publish("service:{}".format(name), (req_id, args))
        self.join(sub)
        return obj.get("val", None)


def main():
    import time
    r = RsdRedis()
    print("pub sub test:")
    r.subscribe("hello", lambda data: print("hello", data))
    r.publish("hello", "world")

    time.sleep(.1)
    r.unsubscribe()

    print("\nservice test:")
    r.service("mul2", lambda x: x * 2)
    val = r.service_call("mul2", 3)
    print(val)

    time.sleep(.1)
    r.unsubscribe()

    print("\ntest finished")


if __name__ == '__main__':
    main()
