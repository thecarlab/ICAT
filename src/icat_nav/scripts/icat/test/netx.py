import networkx as nx
import matplotlib.pyplot as plt

def build_road_graph():
    # Create a new graph
    G = nx.Graph()
    
    # Add intersections as nodes
    G.add_node('A', pos=(0, 0))
    G.add_node('B', pos=(1, 1))
    G.add_node('C', pos=(2, 0))
    G.add_node('D', pos=(1, -1))

    # Add roads connecting intersections as edges (with distance as an attribute)
    G.add_edge('A', 'B', distance=1.4)
    G.add_edge('B', 'C', distance=1.4)
    G.add_edge('C', 'D', distance=1.4)
    G.add_edge('D', 'A', distance=1.4)
    G.add_edge('A', 'C', distance=2.0)
    G.add_edge('B', 'D', distance=2.0)
    
    return G

def draw_road_graph(G):
    pos = nx.get_node_attributes(G, 'pos')
    nx.draw(G, pos, with_labels=True, node_size=700, node_color='skyblue', font_size=15, width=2, edge_color='gray')
    labels = nx.get_edge_attributes(G, 'distance')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    plt.show()

if __name__ == "__main__":
    road_graph = build_road_graph()
    draw_road_graph(road_graph)