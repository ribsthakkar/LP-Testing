import heapq
from collections import deque
from enum import Enum
from functools import total_ordering, lru_cache

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
        self._disabled_edges = dict()

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
            # print(len(q))
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
        # reachability = {nod: self.all_reachable(nod) for nod in self.nodes()}
        # reachability.pop(end)
        reachability = {nod: self.isreachable(nod, end) for nod in temp_g.nodes()}
        # correct = [(1, 21), (21, 41), (41, 61), (61, 62), (62, 82), (82, 102), (102, 103), (103, 104), (104, 105), (105, 85), (85, 86), (86, 87), (87, 107), (107, 106), (106, 126), (126, 146), (146, 166), (166, 165), (165, 145), (145, 144), (144, 143), (143, 123), (123, 122), (122, 142), (142, 141), (141, 161), (161, 162), (162, 163), (163, 183), (183, 203), (203, 202), (202, 222), (222, 221), (221, 241), (241, 242), (242, 262), (262, 263), (263, 283), (283, 303), (303, 323), (323, 324), (324, 325), (325, 326), (326, 346), (346, 347), (347, 367), (367, 368), (368, 388), (388, 389), (389, 390), (390, 370), (370, 369), (369, 349), (349, 348), (348, 328), (328, 329), (329, 309), (309, 289), (289, 290), (290, 270), (270, 250), (250, 249), (249, 269), (269, 268), (268, 288), (288, 287), (287, 267), (267, 266), (266, 286), (286, 285), (285, 265), (265, 245), (245, 225), (225, 226), (226, 246), (246, 247), (247, 227), (227, 228), (228, 208), (208, 207), (207, 187), (187, 188), (188, 189), (189, 169), (169, 149), (149, 129), (129, 130), (130, 110), (110, 90), (90, 89), (89, 88), (88, 68), (68, 67), (67, 47), (47, 46), (46, 26), (26, 6), (6, 7), (7, 8), (8, 28), (28, 48), (48, 49), (49, 29), (29, 30), (30, 10), (10, 11), (11, 31), (31, 51), (51, 50), (50, 70), (70, 71), (71, 72), (72, 92), (92, 93), (93, 73), (73, 74), (74, 54), (54, 34), (34, 33), (33, 32), (32, 12), (12, 13), (13, 14), (14, 15), (15, 35), (35, 36), (36, 37), (37, 57), (57, 58), (58, 38), (38, 18), (18, 19), (19, 39), (39, 59), (59, 60), (60, 80), (80, 79), (79, 99), (99, 100), (100, 120), (120, 119), (119, 139), (139, 159), (159, 158), (158, 157), (157, 137), (137, 136), (136, 156), (156, 176), (176, 196), (196, 195), (195, 175), (175, 155), (155, 154), (154, 174), (174, 173), (173, 193), (193, 192), (192, 212), (212, 213), (213, 214), (214, 215), (215, 235), (235, 255), (255, 254), (254, 274), (274, 273), (273, 272), (272, 252), (252, 251), (251, 271), (271, 291), (291, 292), (292, 293), (293, 313), (313, 314), (314, 334), (334, 333), (333, 332), (332, 352), (352, 372), (372, 392), (392, 393), (393, 394), (394, 374), (374, 375), (375, 395), (395, 396), (396, 376), (376, 356), (356, 336), (336, 337), (337, 338), (338, 339), (339, 340), (340, 360), (360, 359), (359, 358), (358, 357), (357, 377), (377, 397), (397, 398), (398, 399), (399, 400), (400, 1), (1, 21), (21, 41), (41, 61), (61, 62), (62, 82), (82, 102), (102, 103), (103, 104), (104, 105), (105, 85), (85, 86), (86, 87), (87, 107), (107, 106), (106, 126), (126, 146), (146, 166), (166, 165), (165, 145), (145, 144), (144, 143), (143, 123), (123, 122), (122, 142), (142, 141), (141, 161), (161, 162), (162, 163), (163, 183), (183, 203), (203, 202), (202, 222), (222, 221), (221, 241), (241, 242), (242, 262), (262, 263), (263, 283), (283, 303), (303, 323), (323, 324), (324, 325), (325, 326), (326, 346), (346, 347), (347, 367), (367, 368), (368, 388), (388, 389), (389, 390), (390, 370), (370, 369), (369, 349), (349, 348), (348, 328), (328, 329), (329, 309), (309, 289), (289, 290), (290, 270), (270, 250), (250, 249), (249, 269), (269, 268), (268, 288), (288, 287), (287, 267), (267, 266), (266, 286), (286, 285), (285, 265), (265, 245), (245, 225), (225, 226), (226, 246), (246, 247), (247, 227), (227, 228), (228, 208), (208, 207), (207, 187), (187, 188), (188, 189), (189, 169), (169, 149), (149, 129), (129, 130), (130, 110), (110, 90), (90, 89), (89, 88), (88, 68), (68, 67), (67, 47), (47, 46), (46, 26), (26, 6), (6, 7), (7, 8), (8, 28), (28, 48), (48, 49), (49, 29), (29, 30), (30, 10), (10, 11), (11, 31), (31, 51), (51, 50), (50, 70), (70, 71), (71, 72), (72, 92), (92, 93), (93, 73), (73, 74), (74, 54), (54, 34), (34, 33), (33, 32), (32, 12), (12, 13), (13, 14), (14, 15), (15, 35), (35, 36), (36, 37), (37, 57), (57, 58), (58, 38), (38, 18), (18, 19), (19, 39), (39, 59), (59, 60), (60, 80), (80, 79), (79, 99), (99, 100), (100, 120), (120, 119), (119, 139), (139, 159), (159, 158), (158, 157), (157, 137), (137, 136), (136, 156), (156, 176), (176, 196), (196, 195), (195, 175), (175, 155), (155, 154), (154, 174), (174, 173), (173, 193), (193, 192), (192, 212), (212, 213), (213, 214), (214, 215), (215, 235), (235, 255), (255, 254), (254, 274), (274, 273), (273, 272), (272, 252), (252, 251), (251, 271), (271, 291), (291, 292), (292, 293), (293, 313), (313, 314), (314, 334), (334, 333), (333, 332), (332, 352), (352, 372), (372, 392), (392, 393), (393, 394), (394, 374), (374, 375), (375, 395), (395, 396), (396, 376), (376, 356), (356, 336), (336, 337), (337, 338), (338, 339), (339, 340), (340, 360), (360, 359), (359, 358), (358, 357), (357, 377), (377, 397), (397, 398), (398, 399), (399, 400)]

        while True:
            sps = temp_g.dijkstra(start)
            pth = temp_g.pathfromHere(end)
            # depths = temp_g.nonSP_bfs(pth)

            non_disjoint = []
            print("SP", pth)
            print("Iteration:", i, "Current Known LP: ", sps[end])
            if len(pth) == 1:
                return pth, float('inf')
            for idx, n in enumerate(pth):
                if idx != 0:
                    incom = temp_g.incoming(n)
                    merge_w = 0
                    # print(n, incom)
                    alt_incom = None
                    alternates = 0
                    for i_node in incom:
                        if i_node.id == end:
                            continue
                        # print(i_node.adjacent, sps[i_node], i_node, n)
                        if sps[i_node.id] + i_node.adjacent[n] - sps[n.id] >= merge_w:
                            merge_w = max(merge_w, sps[i_node.id] + i_node.adjacent[n] - sps[n.id])
                            alt_incom = i_node
                            alternates += 1
                    edg = (pth[idx-1].id, pth[idx].id)
                    # print(edg)
                    e, d = temp_g.remove_edge(edg)
                    # rec_tree = {nod: temp_g.all_reachable(nod) for nod in filter(lambda x: x != end, temp_g.nodes())}
                    rec_tree = {nod: temp_g.isreachable(nod, end) for nod in temp_g.nodes()}
                    dest_reachable = all(rec_tree[nod] == reachability[nod] for nod in filter(lambda x: x != end, temp_g.nodes()))

                    # rec_tree.pop(end)
                    if float('inf') > merge_w > 0 and temp_g.isreachable(start, alt_incom.id):
                        non_disjoint.append((LinkType.SIMPLE_LINK, merge_w, -alternates, edg))
                    if dest_reachable:
                        non_disjoint.append((LinkType.ALL_NODE_DISCONNECT, d['weight'], -idx, edg))
                    if rec_tree[start]:
                        non_disjoint.append((LinkType.OD_DISCONNECT, d['weight'], -idx, edg))
                    temp_g.add_edge(edg[0], edg[1], d['weight'])

            print("Non-dis", sorted(non_disjoint))
            if not len(non_disjoint):
                for e, w in removed_edges.items():
                    temp_g.add_edge(e[0], e[1], w['weight'])
                return sps[end], pth
            else:
                rem_e = min(non_disjoint)
                print("Rem E", rem_e)
                # if rem_e[3] in correct:
                #     exit(1)
                e, w = temp_g.remove_edge(rem_e[3])
                removed_edges[e] = w
            i += 1
            self.clear_path_labels()

    def longest_v2(self, start, end):
        EDGE_DEX = 4
        self.reset_weights()
        temp_g = self.__copy__()
        # correct = [(1, 2), (2, 3), (3, 13), (13, 23), (23, 22), (22, 32), (32, 33), (33, 34), (34, 35), (35, 45), (45, 46),
        #         (46, 47), (47, 37), (37, 27), (27, 28), (28, 29), (29, 30), (30, 40), (40, 39), (39, 38), (38, 48),
        #         (48, 58), (58, 57), (57, 67), (67, 77), (77, 87), (87, 88), (88, 89), (89, 99), (99, 100)]
        correct = [(1, 21), (21, 41), (41, 61), (61, 62), (62, 82), (82, 102), (102, 103), (103, 104), (104, 105), (105, 85), (85, 86), (86, 87), (87, 107), (107, 106), (106, 126), (126, 146), (146, 166), (166, 165), (165, 145), (145, 144), (144, 143), (143, 123), (123, 122), (122, 142), (142, 141), (141, 161), (161, 162), (162, 163), (163, 183), (183, 203), (203, 202), (202, 222), (222, 221), (221, 241), (241, 242), (242, 262), (262, 263), (263, 283), (283, 303), (303, 323), (323, 324), (324, 325), (325, 326), (326, 346), (346, 347), (347, 367), (367, 368), (368, 388), (388, 389), (389, 390), (390, 370), (370, 369), (369, 349), (349, 348), (348, 328), (328, 329), (329, 309), (309, 289), (289, 290), (290, 270), (270, 250), (250, 249), (249, 269), (269, 268), (268, 288), (288, 287), (287, 267), (267, 266), (266, 286), (286, 285), (285, 265), (265, 245), (245, 225), (225, 226), (226, 246), (246, 247), (247, 227), (227, 228), (228, 208), (208, 207), (207, 187), (187, 188), (188, 189), (189, 169), (169, 149), (149, 129), (129, 130), (130, 110), (110, 90), (90, 89), (89, 88), (88, 68), (68, 67), (67, 47), (47, 46), (46, 26), (26, 6), (6, 7), (7, 8), (8, 28), (28, 48), (48, 49), (49, 29), (29, 30), (30, 10), (10, 11), (11, 31), (31, 51), (51, 50), (50, 70), (70, 71), (71, 72), (72, 92), (92, 93), (93, 73), (73, 74), (74, 54), (54, 34), (34, 33), (33, 32), (32, 12), (12, 13), (13, 14), (14, 15), (15, 35), (35, 36), (36, 37), (37, 57), (57, 58), (58, 38), (38, 18), (18, 19), (19, 39), (39, 59), (59, 60), (60, 80), (80, 79), (79, 99), (99, 100), (100, 120), (120, 119), (119, 139), (139, 159), (159, 158), (158, 157), (157, 137), (137, 136), (136, 156), (156, 176), (176, 196), (196, 195), (195, 175), (175, 155), (155, 154), (154, 174), (174, 173), (173, 193), (193, 192), (192, 212), (212, 213), (213, 214), (214, 215), (215, 235), (235, 255), (255, 254), (254, 274), (274, 273), (273, 272), (272, 252), (252, 251), (251, 271), (271, 291), (291, 292), (292, 293), (293, 313), (313, 314), (314, 334), (334, 333), (333, 332), (332, 352), (352, 372), (372, 392), (392, 393), (393, 394), (394, 374), (374, 375), (375, 395), (395, 396), (396, 376), (376, 356), (356, 336), (336, 337), (337, 338), (338, 339), (339, 340), (340, 360), (360, 359), (359, 358), (358, 357), (357, 377), (377, 397), (397, 398), (398, 399), (399, 400), (400, 1), (1, 21), (21, 41), (41, 61), (61, 62), (62, 82), (82, 102), (102, 103), (103, 104), (104, 105), (105, 85), (85, 86), (86, 87), (87, 107), (107, 106), (106, 126), (126, 146), (146, 166), (166, 165), (165, 145), (145, 144), (144, 143), (143, 123), (123, 122), (122, 142), (142, 141), (141, 161), (161, 162), (162, 163), (163, 183), (183, 203), (203, 202), (202, 222), (222, 221), (221, 241), (241, 242), (242, 262), (262, 263), (263, 283), (283, 303), (303, 323), (323, 324), (324, 325), (325, 326), (326, 346), (346, 347), (347, 367), (367, 368), (368, 388), (388, 389), (389, 390), (390, 370), (370, 369), (369, 349), (349, 348), (348, 328), (328, 329), (329, 309), (309, 289), (289, 290), (290, 270), (270, 250), (250, 249), (249, 269), (269, 268), (268, 288), (288, 287), (287, 267), (267, 266), (266, 286), (286, 285), (285, 265), (265, 245), (245, 225), (225, 226), (226, 246), (246, 247), (247, 227), (227, 228), (228, 208), (208, 207), (207, 187), (187, 188), (188, 189), (189, 169), (169, 149), (149, 129), (129, 130), (130, 110), (110, 90), (90, 89), (89, 88), (88, 68), (68, 67), (67, 47), (47, 46), (46, 26), (26, 6), (6, 7), (7, 8), (8, 28), (28, 48), (48, 49), (49, 29), (29, 30), (30, 10), (10, 11), (11, 31), (31, 51), (51, 50), (50, 70), (70, 71), (71, 72), (72, 92), (92, 93), (93, 73), (73, 74), (74, 54), (54, 34), (34, 33), (33, 32), (32, 12), (12, 13), (13, 14), (14, 15), (15, 35), (35, 36), (36, 37), (37, 57), (57, 58), (58, 38), (38, 18), (18, 19), (19, 39), (39, 59), (59, 60), (60, 80), (80, 79), (79, 99), (99, 100), (100, 120), (120, 119), (119, 139), (139, 159), (159, 158), (158, 157), (157, 137), (137, 136), (136, 156), (156, 176), (176, 196), (196, 195), (195, 175), (175, 155), (155, 154), (154, 174), (174, 173), (173, 193), (193, 192), (192, 212), (212, 213), (213, 214), (214, 215), (215, 235), (235, 255), (255, 254), (254, 274), (274, 273), (273, 272), (272, 252), (252, 251), (251, 271), (271, 291), (291, 292), (292, 293), (293, 313), (313, 314), (314, 334), (334, 333), (333, 332), (332, 352), (352, 372), (372, 392), (392, 393), (393, 394), (394, 374), (374, 375), (375, 395), (395, 396), (396, 376), (376, 356), (356, 336), (336, 337), (337, 338), (338, 339), (339, 340), (340, 360), (360, 359), (359, 358), (358, 357), (357, 377), (377, 397), (397, 398), (398, 399), (399, 400)]

        removed_edges = dict()
        i = 0
        reachability = {nod: self.isreachable(nod, end) for nod in temp_g.nodes()}
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
                    incom = temp_g.incoming(n)
                    merge_w = 0
                    # print(n, incom)
                    alt_incom = None
                    alternates = 0
                    for node in pth:
                        temp_g.disable_node(node)
                    temp_g.reset_weights()
                    for i_node in incom:
                        # if n.id == 400:
                        #     print(i_node.adjacent, new_sp[i_node.id], i_node, n)
                        new_sp = temp_g.dijkstra(start)
                        if temp_g.isreachable(n.id, end, (i_node.id,)) and \
                                float('inf') > new_sp[i_node.id] + i_node.adjacent[n] - sps[n.id] > merge_w and \
                                temp_g.isreachable(pth[idx-1].id, i_node.id, frozenset(x.id for x in pth)):
                            merge_w = max(merge_w, new_sp[i_node.id] + i_node.adjacent[n] - sps[n.id])
                            alt_incom = i_node
                            alternates += 1
                    for node in pth:
                        temp_g.replace_node(node)
                    edg = (pth[idx - 1].id, pth[idx].id)
                    # print(edg)
                    e, d = temp_g.remove_edge(edg)
                    rec_tree = {nod: temp_g.isreachable(nod, end) for nod in temp_g.nodes()}
                    dest_reachable = all(rec_tree[nod] == reachability[nod] for nod in filter(lambda x: x != end, temp_g.nodes()))
                    all_reach = rec_tree == reachability
                    # print('possible edges', edg, merge_w)
                    # rec_tree.pop(end)
                    if float('inf') > merge_w > 0 and temp_g.isreachable(start, alt_incom.id):
                        non_disjoint.append((LinkType.SIMPLE_LINK, -merge_w, -idx, -alternates, edg, dest_reachable, all_reach))
                    elif dest_reachable:
                        non_disjoint.append((LinkType.ALL_NODE_DISCONNECT, d['weight'], idx, -alternates, edg, dest_reachable, all_reach))
                    elif rec_tree[start]:
                        non_disjoint.append((LinkType.OD_DISCONNECT, d['weight'], idx, -alternates, edg, dest_reachable, all_reach))
                    temp_g.add_edge(edg[0], edg[1], d['weight'])
            # print("Non-dis", sorted(non_disjoint))
            done = False
            done2 = False
            first = -1
            second = -1
            sols1 = [(e, e[EDGE_DEX] not in correct) for e in filter(lambda x: x[0] == LinkType.SIMPLE_LINK, sorted(non_disjoint))]
            sols2 = [(e, e[EDGE_DEX] not in correct) for e in filter(lambda x: x[0] == LinkType.ALL_NODE_DISCONNECT, sorted(non_disjoint))]
            sols3 = [(e, e[EDGE_DEX] not in correct) for e in filter(lambda x: x[0] == LinkType.OD_DISCONNECT, sorted(non_disjoint))]
            for idx, e in enumerate(sorted(non_disjoint)):
                if not done2 and e[EDGE_DEX] in correct:
                    print(idx, 'highest incorrect edge is possible', e)
                    done2 = True
                    first = idx
                elif not done and e[EDGE_DEX] not in correct:
                    print(idx, 'Highest non incorrect edge possible', e)
                    done = True
                    second = idx
            if first > -1 and first < second: print("INCORRECT POSSIBLE by ", second - first)
            else: print("CORRECT CHOICE")
            print(sols1)
            print(sols2)
            print(sols3)
            if not len(non_disjoint):
                for e, w in removed_edges.items():
                    temp_g.add_edge(e[0], e[1], w['weight'])
                return sps[end], pth
            else:
                rem_e = min(non_disjoint)
                # non_disjoint.remove(rem_e)
                # rem_e = min(non_disjoint)
                # print("Rem E", rem_e)
                # if rem_e[3] in correct:
                #     print("Solution Link Removed")
                e, w = temp_g.remove_edge(rem_e[EDGE_DEX])
                removed_edges[e] = w
            i += 1
            self.clear_path_labels()

    @lru_cache(1000)
    def longest_v3(self, start, end, ignore=frozenset()):
        temp_g = self.__copy__()

        removed_edges = dict()
        i = 0
        reachability = {nod: self.isreachable(nod, end) for nod in temp_g.nodes()}
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
                    incom = temp_g.incoming(n)
                    merge_w = 0
                    # print(n, incom)
                    alt_incom = None
                    alternates = 0
                    temp_g.disable_node(n)
                    new_sp = temp_g.dijkstra(start)
                    temp_g.replace_node(n)
                    for i_node in incom:
                        # if n.id == 400:
                        #     print(i_node.adjacent, new_sp[i_node.id], i_node, n)
                        if temp_g.isreachable(n.id, end, (i_node.id,)) and float('inf') > new_sp[i_node.id] + i_node.adjacent[n] - sps[n.id] > merge_w and temp_g.isreachable(pth[idx-1].id, i_node.id, (n.id,)):
                            merge_w = max(merge_w, new_sp[i_node.id] + i_node.adjacent[n] - sps[n.id])
                            alt_incom = i_node
                            alternates += 1
                    edg = (pth[idx - 1].id, pth[idx].id)
                    # print(edg)
                    e, d = temp_g.remove_edge(edg)
                    rec_tree = {nod: temp_g.isreachable(nod, end) for nod in temp_g.nodes()}
                    dest_reachable = all(rec_tree[nod] == reachability[nod] for nod in filter(lambda x: x != end, temp_g.nodes()))

                    # rec_tree.pop(end)
                    if float('inf') > merge_w > 0 and temp_g.isreachable(start, alt_incom.id):
                        non_disjoint.append((LinkType.SIMPLE_LINK, merge_w, idx, edg))
                    if dest_reachable:
                        non_disjoint.append((LinkType.ALL_NODE_DISCONNECT, d['weight'], idx, edg))
                    if rec_tree[start]:
                        non_disjoint.append((LinkType.OD_DISCONNECT, d['weight'], idx, edg))
                    temp_g.add_edge(edg[0], edg[1], d['weight'])

            print("Non-dis", sorted(non_disjoint))
            if not len(non_disjoint):
                for e, w in removed_edges.items():
                    temp_g.add_edge(e[0], e[1], w['weight'])
                return sps[end], pth
            else:
                rem_e = min(non_disjoint)
                print("Rem E", rem_e)
                if rem_e[3] in correct:
                    print("Solution Link Removed")
                e, w = temp_g.remove_edge(rem_e[3])
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

    def isreachable(self, start, end, ignored=()):
        if end in ignored:
            return False
        visited = [False] * (self.num+1)
        q = deque([start])
        visited[start] = True
        while len(q):
            cur = q.popleft()
            # print(cur)
            if cur == end:
                return True
            for i in self.neighbors(cur):
                if not visited[i] and i not in ignored:
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

    def __copy__(self):
        g = Graph(self.num)
        for e in self._edges:
            g.add_edge(e[0], e[1], self._edges[e]['weight'])
        return g

    def disable_node(self, nod):
        for e in filter(lambda x: nod.id in x, self._edges):
            self._disabled_edges[e] = self._edges[e]
            self._edges[e] = float('inf')

    def replace_node(self, nod):
        for e in filter(lambda x: nod.id in x, list(self._disabled_edges.keys())):
            self._edges[e] = self._disabled_edges.pop(e)

    def restricted_isreachable(self, n, end, id):
        pass


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