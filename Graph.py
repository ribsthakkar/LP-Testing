import heapq
from collections import deque
from enum import Enum
from functools import total_ordering

@total_ordering
class LinkType(Enum):
    SIMPLE_LINK=0
    ALL_NODE_DISCONNECT=1
    OD_DISCONNECT=2
    def __lt__(self, other):
        if self.__class__ is other.__class__:
            return self.value < other.value
        return NotImplemented

class Graph():
    def __init__(self, nodes):
        self.num = nodes
        self._nodes = list(Node(x) for x in range(self.num + 1))
        self._edges = dict()

    def add_edge(self, start, end, weight):
        self._nodes[start].add_edge(self._nodes[end], weight)
        if (start, end) in self._edges:
            d = self._edges[(start, end)]
            if d['weight'] < weight:
                d['weight'] = weight
        else:
            self._edges[(start, end)] = {'weight': weight}


    def get_edge_data(self, u, v):
        return self._edges[(u,v)]

    def unmark_all(self):
        for node in self._nodes:
            node.unmark()

    def nodes(self):
        return list(node.id for node in self._nodes)

    def neighbors(self, vert):
        return list(node.id for node in self._nodes[vert].adjacent)

    def incoming(self, vert):
        return list(filter(lambda x: vert in x.adjacent, self._nodes))

    def dijkstra(self, start):
        self.reset_weights()
        q = []
        q.append(self._nodes[start])
        self._nodes[start].weight = 0
        heapq.heapify(q)
        while len(q):
            current = heapq.heappop(q)
            # print(current, q)
            if current.visit:
                continue
            current.mark()
            for adj, edge_weight in current.adjacent.items():
                if adj.weight > current.weight + edge_weight:
                    adj.weight = current.weight + edge_weight
                    adj.prev = current
                if adj not in q:
                    heapq.heappush(q, adj)
                else:
                    heapq.heapify(q)
        self.unmark_all()
        d = dict()
        for i in range(len(self._nodes)):
            d[i] = self._nodes[i].weight
            # self._nodes[i].weight = float('inf')
        return d

    def reset_weights(self):
        for i in range(len(self._nodes)):
            self._nodes[i].weight = float('inf')

    def non_zero_dijkstra(self, start):
        q = []
        q.append(self._nodes[start])
        self._nodes[start].weight = 0
        heapq.heapify(q)
        reset = False
        while len(q):
            current = heapq.heappop(q)
            if current.visit:
                continue
            current.mark()
            for adj, edge_weight in current.adjacent.items():
                if not adj.visit:
                    if adj.weight > current.weight + edge_weight:
                        adj.weight = current.weight + edge_weight
                        adj.prev = current
                    heapq.heappush(q, adj)
            if not reset:
                self._nodes[start].weight = float('inf')
                self._nodes[start].unmark()
                reset = True
        self.unmark_all()
        d = dict()
        for i in range(len(self._nodes)):
            d[i] = self._nodes[i].weight
            self._nodes[i].weight = float('inf')
        return d

    def shortest(self, start, end):
        return self.dijkstra(start)[end], self.pathfromHere(end)

    def longest(self, start, end):
        self.reset_weights()
        temp_g = self
        removed_edges = dict()
        i = 0
        reachability = {nod: self.all_reachable(nod) for nod in self.nodes()}
        reachability.pop(end)
        while True:
            sps = temp_g.dijkstra(start)
            pth = temp_g.pathfromHere(end)
            non_disjoint = []
            print("SP", pth)
            print("Iteration:", i, "Current Known LP: ", sps[end])
            if len(pth) == 1:
                return pth, float('inf')
            for idx, n in enumerate(pth):
                if idx != 0:
                    incom = self.incoming(n)
                    merge_w = -float('inf')
                    # print(n, incom)
                    for i_node in incom:
                        # print(i_node.adjacent, sps[i_node], i_node, n)
                        merge_w = max(merge_w, sps[i_node.id] + i_node.adjacent[n] - sps[n.id])
                    edg = (pth[idx-1].id, pth[idx].id)
                    # print(edg)
                    e, d = temp_g.remove_edge(edg)
                    rec_tree = {nod: temp_g.all_reachable(nod) for nod in filter(lambda x: x != end, temp_g.nodes())}
                    dest_reachable = all(rec_tree[nod][end] == reachability[nod][end] for nod in filter(lambda x: x != end, temp_g.nodes()))
                    # rec_tree.pop(end)
                    if dest_reachable and merge_w > 0:
                        non_disjoint.append((LinkType.SIMPLE_LINK, -merge_w, -idx, edg))
                    if dest_reachable:
                        non_disjoint.append((LinkType.ALL_NODE_DISCONNECT, d['weight'], -idx, edg))
                    if rec_tree[start][end]:
                        non_disjoint.append((LinkType.OD_DISCONNECT, d['weight'], -idx, edg))
                    temp_g.add_edge(edg[0], edg[1], d['weight'])

            # print("Non-dis", non_disjoint)
            if not len(non_disjoint):
                for e, w in removed_edges.items():
                    self.add_edge(e[0], e[1], w['weight'])
                return sps[end], pth
            else:
                rem_e = min(non_disjoint)[3]
                # print("Rem E", rem_e)
                e, w = temp_g.remove_edge(rem_e)
                removed_edges[e] = w
            i += 1
            self.clear_path_labels()


    def remove_edge(self, e):
        start, end = e
        self._nodes[start].remove_edge(self._nodes[end])
        return e, self._edges.pop((start, end))

    def pathfromHere(self, current):
        current = self._nodes[current]
        path = [current]
        while current.prev:
            path.append(current.prev)
            current = current.prev
        return list(reversed(path))

    def isreachable(self, start, end):
        visited = [False] * (self.num+1)
        q = deque([start])
        visited[start] = True
        while len(q):
            cur = q.popleft()
            # print(cur)
            if cur == end:
                return True
            for i in self.neighbors(cur):
                if not visited[i]:
                    q.append(i)
                    visited[i] = True
        return False

    def all_reachable(self, start):
        visited = [False] * (self.num+1)
        q = [start]
        visited[start] = True
        while len(q):
            cur = q.pop()
            for i in self.neighbors(cur):
                if not visited[i]:
                    q.append(i)
                    visited[i] = True
        # print(start, visited)
        return visited

    def clear_path_labels(self):
        for node in self._nodes:
            node.prev = None


class Node():
    def __init__(self,x):
        self.id = x
        self.visit = False
        self.adjacent = {}
        self.weight = float('inf')
        self.prev = None

    # Updating adjacent edge if we have a more minimum cost one
    def add_edge(self, other, weight):
        if other not in self.adjacent:
            self.adjacent[other] = weight
        elif self.adjacent[other] > weight:
            self.adjacent[other] = weight

    def remove_edge(self, other):
        # print(self.adjacent)
        self.adjacent.pop(other)

    # Node comparator functions for HeapQ
    def __lt__(self, other):
        return self.weight < other.weight

    def __gt__(self, other):
        return self.weight > other.weight

    # def __eq__(self, other):
    #     return self.weight == other.weight

    def __le__(self, other):
        return self.weight <= other.weight

    def __ge__(self, other):
        return self.weight >= other.weight

    def __ne__(self, other):
        return self.weight != other.weight

    def __hash__(self):
        return self.id

    def mark(self):
        self.visit = True

    def unmark(self):
        self.visit = False

    def __str__(self):
        return str(self.id)

    def __repr__(self):
        return str(self.id) # + ":" + str(self.weight)