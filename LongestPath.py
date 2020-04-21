import Graph as gr
import time

FILE = 'counter-b1.dimacs'

with open(FILE) as f:
    details = list(f.readline().split())
    num_nodes = int(details[2])
    START = 1
    END = 9
    if START < 0 or START > num_nodes or END < 0 or END > num_nodes:
        print("Incorrect Start and End values")
        exit(1)
    num_edges = int(details[3])
    g = gr.Graph(num_nodes)
    count = 0
    while count < num_edges:
        try:
            e = list(f.readline().split())
            if e[0] != 'a':
                continue
            g.add_edge(int(e[1]), int(e[2]), int(e[3]))
        except:
            pass
        count += 1
start_t = time.time()
print(g.shortest(START, END))
end_t = time.time()
print("Shortest Path Time: ", end_t-start_t)

start_t = time.time()
print(g.longest(START, END))
end_t = time.time()
print("Longest Path Time: ", end_t-start_t)