"""
How to cover the nodes with shortest path?
AKA. Traveling Salesman Problem
"""
from topo import get_node_list, get_edge_list, build_graph, get_edge_length, save_edges
import numpy as np
import matplotlib.pyplot as plt



def w_node_list(G):

    w_node_list = {}
    for node in G.nodes():
        adj_nodes = list(G.adj[node])
        weights = [G[node][adj_node]['weight'] for adj_node in adj_nodes]
        # w_node_list.append((node, list(zip(adj_nodes, weights))))
        w_node_list[node] = list(zip(adj_nodes, weights))
    return w_node_list

def coverage_path_planning(w_node_list, start_node):
    path = []
    path.append(start_node)
    # print(w_node_list)
    path_set = set(path)
    while len(path_set) < len(w_node_list):
        adj_nodes = w_node_list[start_node]
        print("adj_nodes: ", adj_nodes)
        min_weight = float('inf')
        min_weight_node = -1
        next_node = -1
        unvisited_adj_nodes = []

        for adj_node, weight in adj_nodes:
            if weight < min_weight:
                min_weight = weight
                min_weight_node = adj_node
            if adj_node not in path_set:
                unvisited_adj_nodes.append((adj_node, weight))

        if len(unvisited_adj_nodes) > 0:
            next_node = -1
            min_w = np.inf
            for node, weight in unvisited_adj_nodes:
                if weight < min_w:
                    min_w = weight
                    next_node = node
        else:
            next_node = min_weight_node
        assert next_node != -1
        print("next_node: ", next_node)
        print("path: ", path)
        print("len of path: ", len(path))

        path.append(next_node)
        path_set = set(path)
        start_node = next_node
    return path


if __name__ == "__main__":
    node_list = get_node_list()
    edge_list = get_edge_list(node_list=node_list)
    G = build_graph(node_list, edge_list)
    w_node_list = w_node_list(G)
    print(w_node_list)
    path = coverage_path_planning(w_node_list, 1)
    print(path)
    print(set(path))

    print("len of path: ", len(path))
    print("len of path set",len(set(path)))

    coverage_edge_list = []
    for i in range(len(path)-1):
        u = path[i]
        v = path[i+1]
        coverage_edge_list.append((u,v,G[u][v]))
    
    for e in coverage_edge_list:
        print("edge: ", e)

    # save_edges("coverage_edges.json", coverage_edge_list)

    save_edges("icat_nav/data/coverage_edges.json", coverage_edge_list)
