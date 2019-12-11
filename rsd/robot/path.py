import queue
from rsd.robot import q
import numpy as np

fully_connected_groups = (
    (q.IDLE, *q.ABOVE_BRICKS, q.ABOVE_CAMERA),
    (q.ABOVE_CAMERA, q.BRICK_DROP_DISCARD_BOX),
    (q.IDLE, *q.BRICK_DROP_ORDER_BOX),
    (q.IDLE, q.MIR_WAYPOINT),
    (q.MIR_WAYPOINT, q.MIR_PUSH_START_ABOVE),
)


def get_all_q():
    all_q = set()
    for group in fully_connected_groups:
        for _q in group:
            all_q.add(_q)
    return list(all_q)


all_q = get_all_q()


def build_graph(groups):
    graph = {}
    for group in groups:
        for q_a in group:
            conn = graph.get(q_a, set())
            for q_b in group:
                if q_a is not q_b:
                    conn.add(q_b)
            graph[q_a] = conn
    return graph


g = build_graph(fully_connected_groups)


def name(_q):
    q_names = [a for a in dir(q) if not a.startswith("__")]
    q_values = [getattr(q, a) for a in q_names]
    i = q_values.index(_q)
    return q_names[i]


def print_path(p):
    print("Path:")
    print(" ", ", ".join([name(q) for q in p]))
    print()


def print_graph(g):
    print("Graph:")
    for q, conn in g.items():
        print("  {}: {}".format(name(q), ", ".join([name(_q) for _q in conn])))
    print()


def find_nearest(_q):
    dq = (np.array(all_q) - _q) * (1, 0.7, 0.5, 0.4, 0.3, 0.2)
    dist = np.linalg.norm(dq, axis=-1)
    i = np.argmin(dist)
    return all_q[i], dist[i]


def find_path(qs, qe):
    # bfs
    assert qs in g and qe in g
    visited = {}
    should_visit = queue.Queue()
    should_visit.put((qs, None))
    res = None
    while not should_visit.empty():
        Q = should_visit.get()
        q, p = Q
        if q is qe:
            res = Q
            break
        if q in visited:
            continue
        for qc in g[q]:
            should_visit.put((qc, Q))
    path = []
    assert res is not None
    while res is not None:
        q, res = res
        path.append(q)
    return reversed(path)


def main():
    from rtde_receive import RTDEReceiveInterface
    from rsd.conf import UR_IP

    rtde_rcv = RTDEReceiveInterface(UR_IP)

    start_q = rtde_rcv.getActualQ()
    n_q, n_dist = find_nearest(start_q)
    p = find_path(n_q, q.IDLE)
    print_path(p)


if __name__ == '__main__':
    main()
