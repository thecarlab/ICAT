import random
import numpy as np
import networkx as nx
import math
from math import pi
import json
from bezier import *
from matplotlib.patches import Circle, Arrow




# G = nx.DiGraph()
# G.add_nodes_from([(1,{"coord":(5,2), "itsc":True}), (2,{"coord":(6,3), "itsc":True})])
# print(G.nodes[1]["coord"])

# G.add_edges_from([(1,2, {"weight":15, "behavior":"turn_left"})])
# print(G.edges[1,2])
WAYPOINT_DISTANCE = 0.5
BEZIER_CONTROL_PARAMETER = 0.6

# def build_graph(filename = "edges.json", is_loading= False):
#     node_list = get_node_list()
#     if is_loading:
#         edge_list = load_edges(filename)
#     else:
#         edge_list = get_edge_list(node_list)
def build_graph(node_list, edge_list):
    G = nx.DiGraph()
    G.add_nodes_from(node_list)
    G.add_edges_from(edge_list)
    return G

def A_star_path(G,ns,ng): # node_start, node_goal
    path = nx.astar_path(G, ns, ng, heuristic=None, weight="weight")
    cost = nx.astar_path_length(G, ns, ng, heuristic=None, weight="weight")
    # print(path)
    return path, cost

def get_node_list():

    node_list= [
        (1,  {"coord":(11.0,3.25,0.), "pre":[(10,'l')], "next":[(2,'s')],"itsc": False}),
        (2,  {"coord":(21,3.25,0.), "pre":[(1,'s')], "next":[(3,'s'),(19,'l')], "itsc": False}),
        (3,  {"coord":(38,3.25,0.), "pre":[(2,'s'),(15,'l')], "next":[(4,'s')], "itsc": False}),
        (4,  {"coord":(49.0,3.25,0.), "pre":[(3,'s')], "next":[(5,'l')], "itsc": False}),
        (5,  {"coord":(56.5,10.3, pi/2), "pre":[(4,'l')], "next":[(6,'s')], "itsc": False}),
        (6,  {"coord":(56.5, 39.6,pi/2), "pre":[(5,'s')], "next":[(7,'l')], "itsc": False}),
        (7,  {"coord":(49.0,47.2, pi), "pre":[(6,'l')], "next":[(8,'s')], "itsc": False}),
        (8,  {"coord":(11.5,47.2, pi), "pre":[(7,'s')], "next":[(9,'l')], "itsc": False}),
        (9,  {"coord":(3.9,39.6, -pi/2), "pre":[(8,'l')], "next":[(10,'s')], "itsc": False}),
        (10,  {"coord":(3.9,10.3,-pi/2), "pre":[(9,'s')], "next":[(1,'l')], "itsc": False}),

        (11,  {"coord":(7.93,10.3,pi/2), "pre":[(27,'r')], "next":[(12,'s')], "itsc": False}),
        (12,  {"coord":(7.93,16.8,pi/2), "pre":[(11,'s')], "next":[(13,'s'),(31,'r')], "itsc": False}),
        (13,  {"coord":(7.93,33.6,pi/2), "pre":[(12,'s'),(35,'r')], "next":[(14,'s')], "itsc": False}),
        (14,  {"coord":(7.93,39.6,pi/2), "pre":[(13,'s')], "next":[(39,'r')], "itsc": False}),

        (15,  {"coord":(28,13,-pi/2), "pre":[(16,'s')], "next":[(28,'r'),(3,'l')], "itsc": False}),
        (16,  {"coord":(28,17,-pi/2), "pre":[(17,'s'),(32,'r'),(37,'l')], "next":[(15,'s')], "itsc": True}),
        (17,  {"coord":(28,33.6,-pi/2), "pre":[(18,'s')], "next":[(16,'s'),(36,'r'),(33,'l')], "itsc": True}),
        (18,  {"coord":(28,38.1,-pi/2), "pre":[(40,'r')], "next":[(17,'s')], "itsc": False}),

        (19,  {"coord":(32,13,pi/2), "pre":[(29,'r'),(2,'l')], "next":[(20,'s')], "itsc": False}),
        (20,  {"coord":(32,17,pi/2), "pre":[(19,'s')], "next":[(21,'s'),(33,'r'),(36,'l')], "itsc": True}),
        (21,  {"coord":(32,33.6,pi/2), "pre":[(20,'s'),(37,'r'),(32,'l')], "next":[(22,'s')], "itsc": True}),
        (22,  {"coord":(32,38.1,pi/2), "pre":[(21,'s')], "next":[(41,'r')], "itsc": False}),

        (23,  {"coord":(52.4,10.3,-pi/2), "pre":[(24,'s')], "next":[(30,'r')], "itsc": False}),
        (24,  {"coord":(52.4,16.8,-pi/2), "pre":[(25,'s'),(34,'r')], "next":[(23,'s')], "itsc": False}),
        (25,  {"coord":(52.4,33.6,-pi/2), "pre":[(26,'s')], "next":[(24,'s'),(38,'r')], "itsc": False}),
        (26,  {"coord":(52.4,39.6,-pi/2), "pre":[(42,'r')], "next":[(25,'s')], "itsc": False}),

        (27,  {"coord":(11.0,7.25, pi), "pre":[(28,'s')], "next":[(11,'r')], "itsc": False}),
        (28,  {"coord":(21,7.25, pi), "pre":[(29,'s'),(15,'r')], "next":[(27,'s')], "itsc": False}),
        (29,  {"coord":(38,7.25, pi), "pre":[(30,'s')], "next":[(28,'s'),(19,'r')], "itsc": False}),
        (30,  {"coord":(49.0,7.25, pi), "pre":[(23,'r')], "next":[(29,'s')], "itsc": False}),

        (31,  {"coord":(13.42,23.5,0.), "pre":[(12,'r')], "next":[(32,'s')], "itsc": False}),
        (32,  {"coord":(21,23.5,0.), "pre":[(31,'s')], "next":[(33,'s'),(16,'r'),(21,'l')], "itsc": True}),
        (33,  {"coord":(38.8,23.5,0.), "pre":[(32,'s'),(20,'r'),(17,'l')], "next":[(34,'s')], "itsc": True}),
        (34,  {"coord":(46,23.5,0.), "pre":[(33,'s')], "next":[(24,'r')], "itsc": False}),

        (35,  {"coord":(13.42,27.2, pi), "pre":[(36,'s')], "next":[(13,'r')], "itsc": False}),
        (36,  {"coord":(21,27.2, pi), "pre":[(37,'s'),(17,'r'),(20,'l')], "next":[(35,'s')], "itsc": True}),
        (37,  {"coord":(38.8,27.2, pi), "pre":[(38,'s')], "next":[(36,'s'),(21,'r'),(16,'l')], "itsc": True}),
        (38,  {"coord":(46,27.2, pi), "pre":[(25,'r')], "next":[(37,'s')], "itsc": False}),

        (39,  {"coord":(11.5,43.25,0.), "pre":[(14,'r')], "next":[(40,'s')], "itsc": False}),
        (40,  {"coord":(21,43.25,0.), "pre":[(39,'s')], "next":[(18,'r'),(41,'s')], "itsc": False}),
        (41,  {"coord":(37.6,43.25,0.), "pre":[(22,'r'),(40,'s')], "next":[(42,'s')], "itsc": False}),
        (42,  {"coord":(49.0,43.25,0.), "pre":[(41,'s')], "next":[(26,'r')], "itsc": False}),
    ]
    return node_list

# ICAT real size 5.22 m * 5.78 m
def get_tuned_node_list():
 
    node_list = get_node_list()
    for node in node_list:
        x, y, yaw = node[1]["coord"]
        node[1]["coord"] = (x*5.78/6, y*5.22/5,yaw)
        # node[1]["coord"][0] = node[1]["coord"][0]
        # node[1]["coord"][1] = node[1]["coord"][1]
    return node_list



def get_edge_list(node_list, interval = 0.05):
    edge_list = []
    for node_id, att in node_list:
        # print(' ')
        # print(" **** Iteration Id is: ",node_id, " attr: ",att)
        coord = att["coord"][:2]
        for next_node, behavior in att['next']:
            next_coord = node_list[next_node-1][1]["coord"][:2]
            # print(" ------->"," next node: " ,next_node, " behavior: ", behavior, "next  coord: ",next_coord)
            edge = build_edge(node_id, next_node, coord, next_coord, behavior, interval = interval)
            n_points = len(edge[2]["waypoints"])
            edge[2]["n_points"] = n_points
            edge_list.append(edge)
    return edge_list

def build_edge(u,v,u_coord,v_coord, behavior,interval = 0.05):
    ux, uy = u_coord
    vx, vy = v_coord
    if behavior == 's':
        # go straight
        waypoints, d = get_straight_waypoints(u_coord, v_coord, distance = interval)
    elif behavior == 'r':
        waypoints, d = get_curve_waypoints(u_coord, v_coord, 'r', distance= interval)
    elif behavior == 'l':
        waypoints, d = get_curve_waypoints(u_coord, v_coord, 'l', distance= interval)
        
    edge = (u,v,{"weight": d,"behavior":behavior, "waypoints":waypoints})
    return edge

def get_straight_waypoints(u_coord, v_coord, distance=0.05):
    # Calculate the distance between u and v
    # print("u: ",u_coord, "  v: ", v_coord)
    ux, uy = u_coord; vx,vy = v_coord
    dx = vx - ux
    dy = vy - uy
    # print("dx: ", dx, "  dy: ", dy)
    d = math.sqrt(dx**2 + dy**2)

    # Calculate the normalized directional vectors
    dir_x = dx / d
    dir_y = dy / d
    # Calculate the yaw angle 
    yaw = math.atan2(dy, dx)
    # Calculate the number of waypoints
    num_waypoints = int(d / distance) + 1

    
    # Generate the waypoints
    waypoints = [(ux + i * distance * dir_x, uy + i * distance * dir_y) for i in range(num_waypoints)]
    
    # Ensure the last waypoint is v for precision issues
    # waypoints[-1] = v_coord
    if dist(waypoints[-1], v_coord) < distance/2:
        waypoints.pop()
    waypoints = [(round(x,3),round(y,3), yaw) for x,y in waypoints]
    
    return waypoints, d

# def get_curve_waypoints(u,v,b,distance):
#     ux, uy = u; vx,vy = v
#     dx = vx-ux; dy = vy-uy
#     points = calculate_control_points(u,v,dx, dy, b) # two control points between u,v
#     # if dx > 0 && dy > 0:
#     t_values = np.linspace(0, 1, 1000)
#     curve_points = [bezier_curve(*points, t) for t in t_values]
#     d = compute_curve_length(curve_points)
#     waypoints = sample_waypoints(curve_points, distance)
#     waypoints = [(round(x,3),round(y,3)) for x,y in waypoints]
#     return waypoints, d


def get_points_from_nodes(node_list):
    pts = []
    angle = []
    for i, node in enumerate(node_list):
        x,y,theta = node[1]["coord"]
        pts.append(np.array([x,y]))
        angle.append(np.array(theta))
    return np.array(pts), np.array(angle)

def get_points_from_edges(edge_list):
    pts = []
    theta = []
    for i, edge in enumerate(edge_list):
        for point in  edge[2]["waypoints"]:
            pts.append(np.array([point[0], point[1]]))
            theta.append(point[2])
    return np.array(pts), np.array(theta)

def get_points_from_path(path, G):
    """
    Return points with [x,y,yaw] from a path
    """
    pts = []
    for n in range(len(path)-1):
        u = path[n]
        v = path[n+1]
        edge_data = G.get_edge_data(u,v)
        pts += edge_data["waypoints"]
    return pts

def calculate_yaw(p1, p2):
    return np.arctan2(p2[1] - p1[1], p2[0] - p1[0])

def get_curve_waypoints(u, v, b, distance):
    ux, uy = u; vx, vy = v
    dx = vx-ux; dy = vy-uy
    points = calculate_control_points(u, v, dx=dx, dy=dy, b=b)  # two control points between u, v

    t_values = np.linspace(0, 1, 1000)
    curve_points = [bezier_curve(*points, t=t) for t in t_values]

    # Sample the curve to get waypoints
    waypoints = sample_waypoints(curve_points, distance)
    
    # Add yaw to the waypoints
    waypoints_with_yaw = []
    for i in range(len(waypoints) - 1):
        yaw = calculate_yaw(waypoints[i], waypoints[i+1])
        waypoints_with_yaw.append((round(waypoints[i][0], 3), round(waypoints[i][1], 3), yaw))
    
    # For the last waypoint, use the yaw of the previous point
    if dist(waypoints[-1],v) >=0.25:
        yaw_last = calculate_yaw(waypoints[-1],v)
        waypoints_with_yaw.append((round(waypoints[-1][0], 3), round(waypoints[-1][1], 3), yaw_last))

    d = compute_curve_length(curve_points)
    
    return waypoints_with_yaw, d






def calculate_control_points(u, v,dx,dy,b,k=BEZIER_CONTROL_PARAMETER):
    ux, uy = u; vx, vy = v
    if dx> 0 and dy > 0:
        if b == 'r':
            p2 = (ux, uy+k*dy)
            p3 = (vx-k*dx, vy)
        elif b == 'l':
            p2 = (ux+k*dx, uy)
            p3 = (vx, vy-k*dy)
    elif dx <0 and dy > 0:
        if b == 'r':
            p2 = (ux+k*dx, uy)
            p3 = (vx, vy-k*dy)
        elif b == 'l':
            p2 = (ux, uy+k*dy)
            p3 = (vx-k*dx, vy)
    elif dx < 0 and dy < 0:
        if b == 'r':
            p2 = (ux,uy+k*dy)
            p3 = (vx-k*dx, vy)
        elif b == 'l':
            p2 = (ux+k*dx, uy)
            p3 = (vx, vy-k*dy)
    elif dx > 0 and dy < 0:
        if b == 'r':
            p2 = (ux + k*dx, uy     )
            p3 = (vx       , vy-k*dy)
        elif b == 'l':
            p2 = (ux, uy+k*dy)
            p3 = (vx-k*dx, vy)
    else:
        raise ValueError (" Check dx or dy, should not be 0!")
    points = [u, p2, p3, v]
    return points

            

def save_edges(filename, edge_list):
    # edge_list = [(u, v, data) for u, v, data in edge_list if "waypoints" in data]
    # Convert the edge list to JSON
    json_data = json.dumps(edge_list, indent=4)

    # Write to file
    with open(filename, 'w') as file:
        file.write(json_data)

def load_edges(filename):
    with open(filename, 'r') as file:
        json_data = file.read()
        
    # Deserialize the JSON data
    edge_list = json.loads(json_data)
    
    return edge_list

def dist(u,v):
    d = None
    if len(u)==len(v):
        if len(u) == 3:
            ux,uy,_ = u
            vx,vy,_ =v
        elif len(u) == 2:
            ux,uy = u
            vx,vy = v
        else:
            raise ValueError ("Length of tuple wrong!")
        dx = vx - ux
        dy = vy - uy
        # print("dx: ", dx, "  dy: ", dy)
        d = math.sqrt(dx**2 + dy**2)
    else:
        raise ValueError ("Two point length not equal!")
    return d

def get_lane_lines(waypoints, road_width=4):
    half_width = road_width / 2.0
    left_lane_points = []
    right_lane_points = []

    for (x, y, yaw) in waypoints:
        # Compute offset for left lane line
        delta_x_left = half_width * math.cos(yaw + math.pi/2)
        delta_y_left = half_width * math.sin(yaw + math.pi/2)
        x_left = x + delta_x_left
        y_left = y + delta_y_left
        
        # Compute offset for right lane line
        delta_x_right = half_width * math.cos(yaw - math.pi/2)
        delta_y_right = half_width * math.sin(yaw - math.pi/2)
        x_right = x - delta_x_right
        y_right = y - delta_y_right
        
        left_lane_points.append((x_left, y_left))
        right_lane_points.append((x_right, y_right))

    return left_lane_points, right_lane_points

def view_topo(ax, node_list, edge_list,if_arrow = False):
    # node_list = get_node_list()
    # edge_list = get_edge_list(node_list=node_list)
    # for edge in edge_list:
    #     print("----------------------")
    #     print("----> ", edge)
    node_list = node_list
    edge_list = edge_list

    # 
    img = load_img('icat.png')

    for node_id, data in node_list:
        x, y, yaw = data["coord"]
        
        # Differentiate color based on "itsc" property
        color = 'green' if data["itsc"] else 'blue'
        
        # Draw the disk patch (circle)
        circle = Circle((x, y), radius=1, color=color, ec="black")  # ec stands for edgecolor
        ax.add_patch(circle)
        
        # Add node ID inside the circle
        ax.text(x, y, str(node_id), ha='center', va='center', color='white')

    for edge in edge_list:
        points = edge[2]["waypoints"]
        #points = [(10, 10), (30, 40), (55, 5), (5, 45)]

        # Unzip the points to separate x and y coordinates for easy plotting
        x_coords, y_coords, yaw = zip(*points)

        # Plotting the points
        plt.scatter(x_coords, y_coords, color='red')  # You can change the color and other properties as needed.
        # Drawing lines connecting the points
        plt.plot(x_coords, y_coords, color='red')  # You can change the color and other properties as needed.

    if if_arrow:
        for edge in edge_list:
            points = edge[2]["waypoints"]
            for x, y, yaw in points:
                arrow_length = 0.5  # Adjust as needed
                # plt.scatter(x, y, color='red')
                dx = arrow_length * np.cos(yaw)
                dy = arrow_length * np.sin(yaw)
                
                arrow = Arrow(x, y, dx, dy, width=0.2, color='red')  # Adjust width and color as needed
                ax.add_patch(arrow)

    # Plotting the image
    # plt.imshow(img, origin='lower', extent=[0, 60, 0, 50])  # The extent parameter sets the axis limits and 'origin' is now set to 'lower'.
    # plt.imshow(img, extent=[0, 60, 0, 50]) 
    # Setting the x and y limits for the axes
    plt.xlim(0, 60)
    plt.ylim(0, 50)

    # Displaying the plot
    # plt.show()
    # return ax

def find_closest_waypoint( x, y, waypoints):
    min_distance = float('inf')
    closest_index = 0

    for i, (wx, wy, _) in enumerate(waypoints):
        distance = math.sqrt((x-wx)**2 + (y-wy)**2)
        if distance < min_distance:
            min_distance = distance
            closest_index = i

    return closest_index
    
def frenet_transform(x, y, waypoints, wpt_dist):
    closest_idx = find_closest_waypoint(x, y, waypoints)
    wx, wy, wyaw = waypoints[closest_idx]

    # Calculate direction vector of the path
    dx_path = math.cos(wyaw)
    dy_path = math.sin(wyaw)

    # Calculate the vector from the waypoint to the car
    dx_car = x - wx
    dy_car = y - wy

    # Project onto the direction vector (for s)
    s_proj = dx_car * dx_path + dy_car * dy_path
    s = closest_idx * wpt_dist + s_proj

    # Project onto the normal of the direction vector (for d)
    dx_norm = -dy_path
    dy_norm = dx_path
    d_proj = dx_car * dx_norm + dy_car * dy_norm
    d = d_proj

    return s, d

def sample_one_node(length):
    node = random.randint(1, length)
    return node

def init_wpts(self):
    WptsBuffer = []
    for i in range(self.n_car):
        path = self.PathBuffer[i].copy()
        waypoints = []
        for n in range(len(path)-1):
            edge_data = self.G.get_edge_data(path[n],path[n+1])
            waypoints += edge_data["waypoints"]
        for k in range(len(waypoints)-1):
            x1, y1, _ = waypoints[k]
            x2, y2, _ = waypoints[k+1]
            if (x1-x2)**2+(y1-y2)**2 <= 0.04:
                print(" Cheking waypoints distance! distance < 0.2: ", waypoints[k], waypoints[k+1])
        WptsBuffer.append(waypoints)
    return WptsBuffer

def plot_cars(car_patches, car_states, car_length, car_width):
    for i in range(len(car_states)):
        x,y,theta = car_states[:3]
        car_patches[i].set_xy((x - car_length / 2 * np.cos(theta) + car_width / 2 * np.sin(theta), 
                y - car_width / 2 * np.cos(theta) - car_length / 2 * np.sin(theta)))
        # car.set_xy((x, y))
        car_patches[i].angle = np.degrees(theta)
    
def get_merge_node():
    node_list = get_node_list()
    edge_list = get_edge_list(node_list)
    G = build_graph(node_list, edge_list)
    
    # (24,  {"coord":(52.4,16.8,-pi/2), "pre":[(25,'s'),(34,'r')], "next":[(23,'s')], "itsc": False}),
    # (25,  {"coord":(52.4,33.6,-pi/2), "pre":[(26,'s')], "next":[(24,'s'),(38,'r')], "itsc": False}),    
    merge_node_list = {} #[(v, [(u,v), (w,v), (z,v)])]
    merge_edge_list = {} #[(u,v): [(w,v), (z,v)]] key = (u,v), value is a list
    for node in node_list:
        id, data = node
        if len(data["pre"]) > 1:
            in_edges = []
            for i in range(len(data["pre"])):
                u = data["pre"][i][0]
                in_edges.append((u,id))
            merge_node_list[id] = in_edges
    for id in merge_node_list:
        edges = merge_node_list[id]
        for edge in edges:
            merge_edge_list[edge] = [t for t in edges if t != edge]
    return merge_node_list, merge_edge_list

def get_diverge_node():
    node_list = get_node_list()
    edge_list = get_edge_list(node_list)
    G = build_graph(node_list, edge_list)
    
    # (24,  {"coord":(52.4,16.8,-pi/2), "pre":[(25,'s'),(34,'r')], "next":[(23,'s')], "itsc": False}),
    # (25,  {"coord":(52.4,33.6,-pi/2), "pre":[(26,'s')], "next":[(24,'s'),(38,'r')], "itsc": False}),    
    diverge_node_list = {} #[(v, [(u,v), (w,v), (z,v)])]
    diverge_edge_list = {} #[(u,v): [(w,v), (z,v)]] key = (u,v), value is a list
    for node in node_list:
        id, data = node
        if len(data["next"]) > 1:
            in_edges = []
            for i in range(len(data["next"])):
                v = data["next"][i][0]
                in_edges.append((id,v))
            diverge_node_list[id] = in_edges
    for id in diverge_node_list:
        edges = diverge_node_list[id]
        for edge in edges:
            diverge_edge_list[edge] = [t for t in edges if t != edge]
    return diverge_node_list, diverge_edge_list 

            
# ***************** Testing *****************************
# # node_list = get_node_list()
# # print(node_list)

# node_list = get_node_list()
# # get_edge_list(node_list)
# # print(node_list)

# wps, d = get_straight_waypoints((21,3.25),(38,3.25))
# print("way points: ", wps, d)
# merge_node_list, merge_edge_list = get_merge_node()
# # print(merge_node_list)
# print("**************")
# for edge in merge_edge_list:
#     print("edge: {}, merging with {}".format(edge, merge_edge_list[edge]))

"""
        (8,  {"coord":(11.5,47.2, pi), "pre":[(7,'s')], "next":[(9,'l')], "itsc": False}),
        (9,  {"coord":(3.9,39.6, -pi/2), "pre":[(8,'l')], "next":[(10,'s')], "itsc": False}),
        (10,  {"coord":(3.9,10.3,-pi/2), "pre":[(9,'s')], "next":[(1,'l')], "itsc": False}),
"""
def get_edge_length(G, edge):
    return G.get_edge_data(edge)["weight"]