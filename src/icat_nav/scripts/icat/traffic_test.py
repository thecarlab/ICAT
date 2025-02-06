from car import *
from traffic_manager import *



WAYPOINT_DISTANCE = 0.5
N_CAR = 20
N_NODE = 42
CAR_PARAM = get_car_param()
CAR_INFO = {"hl":1.775, "hw": 1.0, "amax":3.0, "amin":-3.0, "jerkmax": 10.0} # half length, half width of the car
DT = 0.1
N_LOOP = 1000

car_length = CAR_INFO["hl"]*2
car_width = CAR_INFO["hw"]*2
node_list = get_node_list()
edge_list = get_edge_list(node_list)
G = build_graph(node_list, edge_list)
node_arr = [i for i in range(1, N_NODE+1)]
# Sample nodes for start and goal
nodes = random.sample(node_arr, 2 * N_CAR)
start_nodes = nodes[:N_CAR]
goal_nodes = nodes[N_CAR:]
# start_nodes[0] = 5
# goal_nodes[0] = 6
start_nodes[0] = 16
goal_nodes[0] = 4
start_nodes[1] = 15
goal_nodes[1] = 4
# start_nodes = [21, 37, 28, 7, 39]
# goal_nodes = [19, 5, 4, 18, 36]
# Initalize Cars
cars = []
car_states = []
path_buffer = []
print(" *********************************")
# print("node list: ", node_list)
print("start nodes: ", start_nodes)
print("goal nodes: ", goal_nodes)
for i in range(N_CAR):
    n = start_nodes[i]
    coord = node_list[n-1][1]["coord"]
    print("car {}, start nodes id {}, coord {} ".format(i,n,coord))
    print(coord)
    # Add cars
    car = build_car(i, CAR_PARAM, coord)
    cars.append(car)
    car_states.append(car.state.copy())
    # Add paths
    path = nx.astar_path(G, start_nodes[i], goal_nodes[i], heuristic=None, weight="weight")
    path_buffer.append(path)
 
fig,ax = plt.subplots()
# plt.imshow(img, extent=[0, 60, 0, 50]) 
# Setting the x and y limits for the axes
plt.xlim(0, 60)
plt.ylim(0, 50)
# view_topo(ax, node_list, edge_list, if_arrow=False)
# plt.show()

# point = patches.PathPatch(
#     path=patches.Path([(-0.5, 0.5), (0.5, -0.5), (0, 0), (-0.5, -0.5), (0.5, 0.5)]),
#     facecolor='red',
#     lw=2,
#     edgecolor='black',
# )
# ax.add_patch(point)


car_patches = [Rectangle((cars[i].state[0], cars[i].state[1]), CAR_INFO["hl"]*2, CAR_INFO["hw"]*2, fc='y') for i in range(N_CAR)]
for car_rect in car_patches:
    ax.add_patch(car_rect)
# plt.show()
for i in range(N_CAR):
    x, y, theta = car_states[i][:3]
    # car.set_xy((x - car_length / 2 * np.cos(theta), y - car_length / 2 * np.sin(theta)))
    
    car_patches[i].set_xy((x - car_length / 2 * np.cos(theta) + car_width / 2 * np.sin(theta), 
                y - car_width / 2 * np.cos(theta) - car_length / 2 * np.sin(theta)))
    # car.set_xy((x, y))
    car_patches[i].angle = np.degrees(theta)
# plt.show()


TM = TrafficManager(node_list=node_list, edge_list=edge_list, G = G, n_car = N_CAR,
                    car_states=car_states, car_info = CAR_INFO, start_nodes=start_nodes, goal_nodes=goal_nodes,
                    wpts_dist=WAYPOINT_DISTANCE)
# # 
# for edge_wpts in TM.WptsBuffer[0]:
#     print("edge_wpts: ", edge_wpts)
# print(" car[0] state: ",cars[0].state)
# edge_wpts = [(21.0, 43.25, 0.0), (21.5, 43.25, 0.0), (22.0, 43.25, 0.0), (22.5, 43.25, 0.0), (23.0, 43.25, 0.0), (23.5, 43.25, 0.0)]
# # TM.localize_to_road(cars[0].state, car_id = 0)

# TM.localize_to_road(cars[0].state, car_id = 0)

# s,d, in_edge, closest_index, closest_point = TM.localize_to_road([37.901,43.784, 0.,0.,0.], car_id = 0)
# print("In edge: ", in_edge)

# [(37.6, 43.25, 0.0), (38.1, 43.25, 0.0), (38.6, 43.25, 0.0), (39.1, 43.25, 0.0), (39.6, 43.25, 0.0),
import time
stime = time.time()
sim_ctr = 0
for n_loop in range(N_LOOP):
    sim_ctr +=1 
   
    TM.traffic_state_update(car_states)
    # plot_cars(car_patches, car_states,CAR_INFO["hl"]*2, CAR_INFO["hw"]*2)
    for i in range(N_CAR):
        x, y, theta = car_states[i][:3]
        # car.set_xy((x - car_length / 2 * np.cos(theta), y - car_length / 2 * np.sin(theta)))
        
        car_patches[i].set_xy((x - car_length / 2 * np.cos(theta) + car_width / 2 * np.sin(theta), 
                    y - car_width / 2 * np.cos(theta) - car_length / 2 * np.sin(theta)))
        # car.set_xy((x, y))
        car_patches[i].angle = np.degrees(theta)
    # plt.draw()
        # ahead_point = TM.StateBuffer[i]["ahead_point"]
        # plt.scatter(ahead_point[0], ahead_point[1], marker='x', color='red', s=100)
        # print("Robot {} in edge: {}, with path {} ".format( i, TM.StateBuffer[i]["in_edge"], TM.PathBuffer[i] ))

    # plt.clf()
    # plt.xlim(0, 60)
    # plt.ylim(0, 50)
    # plot_topo(node_list,edge_list, ax, if_arrow=False)
    # trajbuffer = TM.get_traj_buffer()
    # # for traj in trajbuffer:
    # #     print("********************* Traj: *****************")
    # #     print(traj)
    # #     print("*******************************")
    # #     plot_traj(traj)
    # plot_cars(trajbuffer, car_length, car_width)
    
         


    # plt.pause(0.01)

    # # car_states = []
    # for i in range(N_CAR):
    #     cmd = cars[i].move_base(TM.CommandBuffer[i])
    #     print("cmd is: ", cmd)
    #     print("car states before update: ", car_states[i])
    #     cars[i].state_update(cmd)
    #     # car_states.append(cars[i].state)
    #     print("car states after update: ", car_states[i])

    """ 
    Car state update
    """
    car_states=[]
    Tbuffer = TM.get_traj_buffer()
    for id in range(N_CAR):
        car_states.append(Tbuffer[id][1])

etime = time.time()
print(" running {} loops, cost time {}, average time cost {} ".format(N_LOOP, etime-stime, (etime-stime)/N_LOOP))