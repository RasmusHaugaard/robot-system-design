import requests
import json
import time
from rsd.packml.packml import PackMLState as S

MES_PACKML_STATES = {
    S.IDLE: "PML_Idle",
    S.EXECUTE: "PML_Execute",
    S.COMPLETE: "PML_Complete",
    S.HELD: "PML_Held",
    S.SUSPENDED: "PML_Suspended",
    S.ABORTED: "PML_Aborted",
    S.STOPPED: "PML_Stopped",

}


class Mes:
    def __init__(self, server_url):
        self.server_url = server_url

    def get_orders(self):
        res = requests.get("{}/orders".format(self.server_url))
        assert res.status_code == 200
        return json.loads(res.content)['orders']

    def take_order(self, order):
        res = requests.put("{}/orders/{}".format(self.server_url, order['id']))
        if res.status_code != 200:
            return False
        return json.loads(res.content)['ticket']

    def complete_order(self, order, ticket):
        res = requests.delete("{}/orders/{}/{}".format(self.server_url, order['id'], ticket))
        _ = res.content  # to close conn
        self.log_order_done(order['id'])
        return res.status_code == 204  # no content

    def take_a_ready_order(self):
        while True:
            ready_orders = [o for o in self.get_orders() if o['status'] == 'ready']
            if not ready_orders:
                print("Currently no ready orders.")
                time.sleep(3)
                continue
            order = ready_orders[0]
            ticket = self.take_order(order)
            if not ticket:
                print("Order was already taken. Trying a different order.")
                continue
            self.log_order_start(order['id'])
            return order, ticket

    def log_state(self, state):
        if state in MES_PACKML_STATES:
            body = {
                "cell_id": 9,
                "event": MES_PACKML_STATES[state],
                "comment": "",
            }
            res = requests.post("{}/log".format(self.server_url), json=body)
            assert res.status_code == 201  # Created

    def log_order_start(self, id):
        body = {
            "cell_id": 9,
            "event": "Order_Start",
            "comment": "order {}".format(id),
        }
        res = requests.post("{}/log".format(self.server_url), json=body)
        assert res.status_code == 201  # Created

    def log_order_done(self, id):
        body = {
            "cell_id": 9,
            "event": "Order_Done",
            "comment": "order {}".format(id),
        }
        res = requests.post("{}/log".format(self.server_url), json=body)
        assert res.status_code == 201  # Created


def main():
    import rsd.conf as conf
    mes = Mes(conf.MES_SERVER_URL)
    orders = mes.get_orders()
    print(orders)

    order, ticket = mes.take_a_ready_order()
    # prepare order
    mes.complete_order(order, ticket)
    mes.log_state(S.COMPLETE)


if __name__ == '__main__':  # If this file is run directly. (Not if this file is imported in another file)
    main()
