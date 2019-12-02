import requests
import json
import time


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
        assert res.status_code == 204  # no content

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
            return order, ticket


def main():
    mes = Mes("http://127.0.0.1:5000")
    orders = mes.get_orders()
    print(orders)

    order, ticket = mes.take_a_ready_order()
    # prepare order
    mes.complete_order(order, ticket)


if __name__ == '__main__':  # If this file is run directly. (Not if this file is imported in another file)
    main()
