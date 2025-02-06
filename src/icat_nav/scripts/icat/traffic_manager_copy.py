
import numpy as np
import networkx as nx
import random
from topo import *
from car import build_car,get_car_param
from math import pi, sin, cos, atan2
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import bisect
from quintic import quintic_1d_plan
from plot import *
class TrafficManager:


    def __init__(self, node_list, edge_list, G, n_car, car_states, car_info, start_nodes, goal_nodes = [], wpts_dist = 0.5, kwards = {}):
        self.wpts_dist = wpts_dist
        self.node_list = node_list
        self.node_range = [i+1 for i in range(len(node_list))]
        self.edge_list = edge_list
        self.G = G
        self.n_car = n_car
        self.start_nodes = start_nodes
        self.goal_nodes = goal_nodes
        self.car_info = car_info
        self.look_ahead_dist = 2.0+car_info["hl"]  # For wpts tracking
        self.check_ahead_dist = 10.0 # For car following
        self.safe_clearance = 1.0 + 2*car_info["hl"]  # inter-car center clearance 
        self.constant_speed = 5.0 
        self.head_time = 1.0  
        self.merge_node_list, self.merge_edge_list = get_merge_node()
        self.diverge_node_list, self.diverge_edge_list = get_diverge_node() 
        self.PathBuffer = self.init_path()            # contains node like [[1,2], [5,10,12]]
        for i in range(self.n_car):
            self.check_path(i,if_update_wpts = False)
        self.LocalWptsBuffer = [np.empty(0) for i in range(self.n_car)]
        self.WptsBuffer = self.init_wpts()            # contains waypoints like [ [(x,y,yaw)...], [], [] ]
        self.Sbuffer = [np.empty(0) for i in range(self.n_car)]                             # contains s coords along the path       
        self.TrajBuffer = [np.empty(0) for i in range(self.n_car)]
        self.StateBuffer = self.init_states(car_states)
        self.FrenetBuffer = []
        self.EdgeOccupancy = []
        self.DistBuffer = {}
        self.BehaviorBuffer =["speed keeping" for i in range(self.n_car)]
        self.CommandBuffer = []
        self.dt = 0.1

    def init_states(self, states):
        StateBuffer = []
        assert self.n_car == len(states), " Car number not equal to len of states!"
        for i in range(self.n_car):
            car_state = {}
            x,y,yaw,v,w = states[i]
            car_state["current_point"] = (x,y,yaw)
            car_state["linear_speed"] = v
            car_state["angular_speed"] = w
            car_state["ahead_point"] = (x,y,yaw)
            car_state["ahead_in_edge"] = (self.PathBuffer[i][0],self.PathBuffer[i][1])
            car_state["ahead_index"] = 0
            StateBuffer.append(car_state)
        return StateBuffer


    def init_path(self):
            PathBuffer = []
            for i in range(self.n_car):
                path = nx.astar_path(self.G, self.start_nodes[i], self.goal_nodes[i], heuristic=None, weight="weight")
                PathBuffer.append(path)

            return PathBuffer

    def init_wpts(self):
        WptsBuffer = []
        for i in range(self.n_car):
            path = self.PathBuffer[i].copy()
            waypoints = []
            local_wpts = []
            for n in range(len(path)-1):
                edge_data = self.G.get_edge_data(path[n],path[n+1])
                waypoints.append(edge_data["waypoints"])
                local_wpts += edge_data["waypoints"]
            self.LocalWptsBuffer[i] = np.array(local_wpts)
            WptsBuffer.append(waypoints)
        return WptsBuffer
    
    def init_s_path(self):
        """ three sections for s path"""
        for i in range(self.n_car):
            self.update_s_path(i)
    
    def update_s_path(self,id):
        Spath = np.empty(0)
        former_s = 0
        for i in range(3): 
            new = np.array([0.5 * n for n in range(len(self.WptsBuffer[id][i]))])+former_s
            Spath = np.concatenate((Spath, new))
            former_s += self.get_edge_length((self.PathBuffer[id][i],self.PathBuffer[id][i+1] )) 
            print(" Path edge: {}, number of wpts: {}, edge lengdth: {} ".format((self.PathBuffer[id][i],self.PathBuffer[id][i+1]), 
                    len(self.WptsBuffer[id][i]),self.get_edge_length((self.PathBuffer[id][i],self.PathBuffer[id][i+1])) ))
        self.Sbuffer[id] = Spath
        # print("updated s path of id: {}, s_path: {}".format(id, self.Sbuffer[id]))
        # print("bisect 78: ", bisect.bisect(self.Sbuffer[id], 79.0))
        # print(" index value: ", self.Sbuffer[id][bisect.bisect(self.Sbuffer[id], 79.0)])
        # assert 1==2, " in update_s_path"


    def update_path_wpts(self, path, i):
        waypoints = []
        local_wpts = []
        for n in range(len(path)-1):
            edge_data = self.G.get_edge_data(path[n],path[n+1])
            waypoints.append(edge_data["waypoints"])
            local_wpts += edge_data["waypoints"]
        self.WptsBuffer[i] = waypoints
        self.LocalWptsBuffer[i] = np.array(local_wpts)
        
    def check_path(self, id, if_update_wpts = False):
        """
        Check if the path contains more than two nodes,
        if not, append another node to replan the path.
        """
        path = self.PathBuffer[id]
        while len(path) < 4:
            print(" ***************************** In check path, doing adding")
            new_goal_node = sample_one_node(len(self.node_list))
            while new_goal_node in path:
                new_goal_node = sample_one_node(len(self.node_list))
            self.goal_nodes[id] = new_goal_node
            path = [path[0]] + nx.astar_path(self.G, path[1], new_goal_node, heuristic=None, weight="weight")
            self.PathBuffer[id] = path
        if if_update_wpts:
            self.update_path_wpts(path, id)
            self.update_s_path(id)

    def localize_to_road(self, state, car_id):
        # Make sure no change to path
        in_edge = (-1, -1)
        path = self.PathBuffer[car_id]
        # print("path: ", path)
        # print("car state: ",state)
        for n in range(len(path)-1):
            edge_points = self.G.get_edge_data(path[n],path[n+1])["waypoints"]
            # print("section points: ", edge_points)
            closest_index = find_closest_waypoint(state[0], state[1], edge_points)
            closest_point = edge_points[closest_index]
            s,d = frenet_transform(state[0], state[1], edge_points)
            if s < 0 :
                # in_edge = (-1, path[n])
                in_edge = (path[n], path[n+1])
                # print(" In edge: ", path[n],path[n+1])
                return s,d, in_edge, closest_index, closest_point
            if s >= 0 and s < self.G.get_edge_data(path[n],path[n+1])["weight"]:
                in_edge = (path[n], path[n+1])
                # print(" In edge: ", path[n],path[n+1])
                return s,d,in_edge, closest_index, closest_point
            
            print("the closest index: ", closest_index)
            print(" print s,d : ", s,d)
        if in_edge == (-1,-1):
            print("Cannot localize error")
            print("car id , ", id)
            print("car's path: ", path)
            print("car state: ", self.StateBuffer[car_id])
            raise ValueError (" Cannot localize to the path !")
      
    def path_pop(self, in_edge, car_id, if_pop_wpts= True):
        path = self.PathBuffer[car_id]
        wpts = self.WptsBuffer[car_id]
        if in_edge[0] in path:
            index = path.index(in_edge[0])
            if index > 0:
                orig_len = len(wpts)
                del path[:index]
                self.PathBuffer[car_id] = path
                if if_pop_wpts:
                    del wpts[:index]
                    self.WptsBuffer[car_id] = wpts
                    assert len(self.WptsBuffer[car_id])!=orig_len, "Fail to pop out passed waypoints!"
        else:
            # print("car id , ", car_id)
            # print("car's path: ", path)
            # print("car in edge: ", in_edge)
            # print("car state: ", self.StateBuffer[car_id])
            raise ValueError (" Edge not in the path, in path state update!")

    def get_look_ahead_point(self,clst_index, id):
        K = 0.1
        u,v,w,z = self.PathBuffer[id][:4]
        ahead_in_edge = (u,v)
        current_speed = self.StateBuffer[id]["linear_speed"]
        ahead_dist = self.look_ahead_dist + K * current_speed
        n_ahead_indices = round(ahead_dist/self.wpts_dist)
        n1 = self.G.get_edge_data(u,v)["n_points"]
        ahead_points = self.WptsBuffer[id][0]+self.WptsBuffer[id][1]+self.WptsBuffer[id][2]
        ahead_index = clst_index + n_ahead_indices
        if clst_index + n_ahead_indices >= n1-1:
            ahead_in_edge = (v,w)
            ahead_index = ahead_index - n1
        # if clst_index+n_ahead_indices > len(ahead_points):
        #     print("uvw: ", u,v,w  )
        #     print(" index len: ",clst_index, n_ahead_indices )
        #     print("ahead points len: ", ahead_points)
        ahead_point = ahead_points[clst_index+n_ahead_indices]
        return ahead_in_edge, ahead_index, ahead_point
                     
            
    """          
    car_state["current_point"] = (x,y,yaw)
    car_state["linear_speed"] = v
    car_state["angular_speed"] = w
    car_state["ahead_point"] = (x,y,yaw)
    """    

    def state_update(self,car_states):
        sd_coords = []
        occupied_edges ={}
        for i in range(self.n_car):
            s,d,in_edge,clst_index, clst_point = self.localize_to_road(car_states[i],i)
            x,y,yaw,v,w = car_states[i]
            self.path_pop(in_edge,car_id=i)
            self.check_path(id=i, if_update_wpts=True)
            if dist(clst_point,self.StateBuffer[i]["ahead_point"]) <= 1.+ self.car_info["hl"]:
                ahead_in_edge, ahead_index, ahead_point = self.get_look_ahead_point(clst_index, i)
                self.StateBuffer[i]["ahead_in_edge"] = ahead_in_edge
                self.StateBuffer[i]["ahead_index"] = ahead_index
                self.StateBuffer[i]["ahead_point"] = ahead_point
            self.StateBuffer[i]["current_point"] = (x,y,yaw)
            self.StateBuffer[i]["linear_speed"] = v
            self.StateBuffer[i]["angular_speed"] = w
            self.StateBuffer[i]["sd"] = (s,d)
            self.StateBuffer[i]["in_edge"] = in_edge
            self.StateBuffer[i]["clst_index"] = clst_index
            self.StateBuffer[i]["clst_point"] = clst_point
 
            sd_coords.append((s,d))
            if in_edge not in occupied_edges:
                occupied_edges[in_edge] = [(i,s,d)]
            else:
                occupied_edges[in_edge].append((i,s,d))
        for in_edge in occupied_edges:
            occupied_edges[in_edge] = sorted(occupied_edges[in_edge], key=lambda x: x[1])
        self.FrenetBuffer = sd_coords
        self.EdgeOccupancy = occupied_edges
        self.car_dist_update()

    def get_next_dist(self,id):
        """
        Check next edge for ditance measurements.
        """
        u,v,w = self.PathBuffer[id][:3]
        s = self.FrenetBuffer[id][0]
        d2v = self.G.get_edge_data(u,v)["weight"]-s
        if (v,w) in self.EdgeOccupancy:
            if len(self.EdgeOccupancy[(v,w)])>1:
                sorted_data = sorted(self.EdgeOccupancy[(v,w)], key=lambda x: x[1])
                ds = sorted_data[0][1]
                leading_id = sorted_data[0][0]
            else:
                ds = self.EdgeOccupancy[(v,w)][0][1]
                leading_id = self.EdgeOccupancy[(v,w)][0][0]
        else:
            ds = self.G.get_edge_data(v,w)["weight"]
            leading_id = -1
        d = d2v + ds
        return (leading_id,d)

    def car_dist_update(self):
        """
        DistBuffer stores (leading id, distance)
        """
        self.DistBuffer = {}
        for in_edge in self.EdgeOccupancy:
            if len(self.EdgeOccupancy[in_edge])>1:
                sorted_data = sorted(self.EdgeOccupancy[in_edge], key=lambda x: x[1])
                for i in range(len(sorted_data)-1):
                    distance = sorted_data[i+1][1] - sorted_data[i][1]
                    id = sorted_data[i][0]
                    leading_id = sorted_data[i+1][0]
                    self.DistBuffer[id] = (leading_id,distance)
                # first car
                id = sorted_data[-1][0]
                self.DistBuffer[id] = self.get_next_dist(id)
            else:
                id = self.EdgeOccupancy[in_edge][0][0]
                if id not in self.DistBuffer:
                    self.DistBuffer[id] = self.get_next_dist(id)
                else:
                    raise ValueError ("ID repeated in DistBuffer!")
    def get_in_edge(self,id):
        return self.StateBuffer[id]["in_edge"]

    def get_edge_length(self, edge):
        return self.G.get_edge_data(edge[0],edge[1])["weight"]

    """
    def check_merge_diverge(self,id):
        in_edge = self.get_in_edge(id)
        if in_edge in self.merge_edge_list:
            intention_edges = self.merge_edge_list[in_edge]
            for edge in intention_edges:
                if 
    """
    def merge_logic(self):
        """
        State machine:
        If in merge/diverge checking area:
            do Check merging/diverging car:
                if is merging/diverging car:
                    stop
                else no car:
                    check on path car:
                        if is on path car:
                            if close:
                                stop
                            else not close:
                                following
        """
        # edges_have_checked = []
        # for occupied_edge in self.EdgeOccupancy:
        #     if occupied_edge in edges_have_checked:
        #         continue
        #     else:
        #         if occupied_edge in self.merge_edge_list:
        #             # check far or close
        #             other_edges = self.merge_edge_list[occupied_edge]
        #             for other_edge in other_edges:
        #                 if other_edge in self.EdgeOccupancy:
        #                     (i,s,d) = self.EdgeOccupancy[other_edge][-1]
        # 直接从merge点查找
        # for merge_node in self.merge_node_list:
        for merge_node in self.merge_node_list:
            edges = self.merge_node_list[merge_node]
            d2p_list = []
            for edge in edges:
                id, s = self.check_merge_edge(edge)
                if id == -1:
                    continue
                else:
                    l = self.get_edge_length(edge)
                    d2p = l-s # distance to merge point
                    d2p_list.append((d2p, id))
            if len(d2p_list) > 0:
                d2p_list = sorted(d2p_list, key=lambda x: x[0]) # sort with d2p

    def acc_traj_plan(self, id):
        """
        Only consider car following logic, where speed keeping logic is included.
        1. Reference path: waypoints [] []
        2. S coordinate: current sc,
        3. planned points s coords, [st0, st1, st2,...., stn]
        4. S buffer associated to waypoints
        5. search cooresponding (x0,y0)， （x1, x2) ...
        for each point with st + sc, search index in s waypoints, 
        多出的ds, 由xi,yi,+ cos(yawi) 等计算出 x,y, yaw
                
        """  
        ss = self.StateBuffer[id]["sd"][0]              # current s coord
        vs = self.StateBuffer[id]["linear_speed"]       # current speed assume perfectly on the road direction
        a_s =  0.0
        se, ve, a_e, minT, maxT, logic = self.calc_acc_end_state(id)
        leading_id, dist = self.DistBuffer[id]
        print("checking backward: ", id)
        print(" dist: ", dist, " ID: ", id)
        print("leading ID: ", leading_id)
        print("dist checking: ", self.DistBuffer[id])
        print("EGO state: ", self.StateBuffer[id])
        print("Leading state: ", self.StateBuffer[leading_id])
        print("ss: ",ss)
        print("se: ", se)
        print("vs: ", vs)
        print("ve: ", ve)
        print("logic: ", logic)
        assert ss <= se, "End point is backward!!!"
        # then we plan a trajectory along s coord
        Ttime, Ts, Tv, Ta, Tj = quintic_1d_plan(ss, vs, a_s, se,ve, a_e,self.car_info["amax"], self.car_info["jerkmax"],self.dt, minT, maxT)  
        Traj= self.calc_traj_from_s(id,Ts,Tv)
        self.TrajBuffer[id] = Traj

    def calc_traj_from_s(self, id, Ts, Tv):
        Txyyaw = []
        Traj = []
        # print("ID: ",id , " ***************")
        # print("Ts: ", Ts)
        # print("Tv: ", Tv)
        assert len(Ts) == len(Tv), "s trajectory length not equal to speed length!"
        for i in range(len(Ts)):
            if Ts[i] >= self.Sbuffer[id][-1]:
                assert Ts[i] < self.Sbuffer[id][-1], "Ts value beyond the s value in buffer"
                break
            index = bisect.bisect(self.Sbuffer[id],Ts[i])-1
            ds = Ts[i] - self.Sbuffer[id][index]
            print("bigger than s checking")
            print("Car ID: {}, bisect index {}, Ts {}, Sbuffer[id][index] {}, self.LocalWptsBuffer[id]{}".format(id,index, Ts, self.Sbuffer[id][index],self.LocalWptsBuffer[id] ))
            assert ds >= 0, "index value bigger than s_value!"
            px, py, yaw = self.LocalWptsBuffer[id][index]
            x = px + ds*cos(yaw)
            y = py + ds*sin(yaw)
            Txyyaw.append((x,y,yaw))
        for k in range(len(Txyyaw)-1):
            x, y, yaw = Txyyaw[k]
            yaw_next = Txyyaw[k+1][2]
            # yaw_next = yaw + w*DT
            v = Tv[k]
            w = (yaw_next - yaw)/self.dt
            Traj.append((x,y,yaw,v,w))
        Traj.append((*Txyyaw[-1],Tv[-1], w))
        assert len(Traj) == len(Ts), "final path length not equal to Ts! "
        return np.array(Traj)



    def calc_acc_end_state(self, id):
        TAU = 0.2 # a parameter to tune headway increment with leading car speed increasing,
        leading_id, dist = self.DistBuffer[id]
        # no leading cars in the current and the next section or far way
        if leading_id == -1 or dist >= self.check_ahead_dist: 
            se = self.FrenetBuffer[id][0] + self.check_ahead_dist
            ve = self.constant_speed # desired speed
            a_e = 0.0
            minT = self.check_ahead_dist / (1.2 * self.constant_speed) # overspeed no more than 20%
            maxT = self.check_ahead_dist / (self.constant_speed/2)
            logic = "too far"
        else:
            if dist > self.safe_clearance:
                # There is a leading car, do Adaptive Cruise Control logic
                # starget(t) := slv(t) − [D0 + τ s˙lv(t)], from the paper Optimal Trajectory Generation for Dynamic Street Scenarios in a Frene´t Frame
                # safe clearance, if lv = 0, 1m, lv=5,2m  end speed should be 0 m/s to assure a full stop
                leading_v = self.StateBuffer[leading_id]["linear_speed"]
                print("ego position: ", self.StateBuffer[id]["sd"])
                print(" leading car position: ", self.FrenetBuffer[id][0] + dist)
                # print(" leading car pose get: ", self.StateBuffer[leading_id]['sd'])
                se = self.FrenetBuffer[id][0] + max((dist + TAU* leading_v - self.safe_clearance ),0.)
                ve = 0.0 # ensure a full stop at the tracking end point
                a_e = 0.0 # stop smoothly
                minT = dist /(1.2 * self.constant_speed) # relax a little
                maxT = dist / (self.constant_speed/4)    # enlarge the time area for stable search
                logic = "following"
            else: # emergency brake!
                a_brake = self.car_info["amax"]
                v = self.StateBuffer[id]["linear_speed"]
                t = max(v/a_brake,self.dt)
                ds = max(v*(t+self.dt) - 0.5*a_brake*t**2,0.0)
                se = self.FrenetBuffer[id][0] + ds
                ve = 0.
                a_e = 0.
                minT = max(t-2*self.dt, self.dt)
                maxT = t+5*self.dt
                print("*********** Emergency Brake! *************")
                print(" dist: ", dist, " ID: ", id)
                print("leading ID: ", leading_id)
                print("dist checking: ", self.DistBuffer[id])
                print("EGO state: ", self.StateBuffer[id])
                print("Leading state: ", self.StateBuffer[leading_id])
                print("v: {}, a: {}, t: {}, ds: {}".format(v,a_brake,t,ds))
                print("a_brake: ", a_brake)
                logic = "brake"
                
        return se, ve, a_e, minT, maxT, logic




    # def check_merge_edge(edge):
    #     if edge in self.EdgeOccupancy:
    #         id,s,d = self.EdgeOccupancy[edge][-1]
    #         return id, s
    #     else:
    #         return -1, 0.

    # def plan_car_following(self, id):
    #     leading_id = self.DistBuffer[id][0]
    #     ego_v = self.StateBuffer[id]["linear_speed"]
    #     leading_v = self.StateBuffer[leading_id]["linear_speed"]
    #     desired_v = ego_v
    #     """
    #     This is car following logic.
    #     """
    #     if leading_id == -1:
    #         desired_v = self.constant_speed
    #     else:
    #         d = self.DistBuffer[id][1]
    #         if d <= self.safe_clearance + self.car_info["hl"]*2:
    #             desired_v = 0.0
    #         else:                               
    #             total_dist = d - self.safe_clearance + self.car_info["hl"]*2
    #             acc_speed = total_dist + self.head_time * (leading_v - ego_v) /self.head_time
    #             desired_v = max(acc_speed, self.constant_speed)
    #     current_point = self.StateBuffer[id]["current_point"]
    #     looking_point = self.StateBuffer[id]["ahead_point"]
    #     s,d = self.StateBuffer[id]["sd"]
    #     return current_point, looking_point, s,d, ego_v , desired_v 


    # def excute_plan(self, plan):
    #     """ Pure pursuit Controller """
    #     Kp = 0.45
    #     current_point, looking_point,s,d, ego_v, disired_v = plan
    #     a = disired_v - ego_v
    #     if a >= 0:
    #         a = min(a, self.car_info["amax"])
    #     if a < 0:
    #         a = max(a, self.car_info["amin"])
    #     planned_v = ego_v+a
    #     looking_yaw = looking_point[2]
    #     ego_yaw = current_point[2]
    #     dangle1 = 0 - d
    #     dangle2 = self.to_pi(looking_yaw - ego_yaw)
    #     # print("following yaw: ", round(looking_yaw,3), "  ego yaw: ", round(ego_yaw,3))
    #     # print(" ******* in Control", round(dangle1,3), round(dangle2,3))
    #     planned_w = Kp*(dangle1+dangle2)
    #     return planned_v, planned_w

    def trajectory_update(self):
        for id in range(self.n_car):
            self.acc_traj_plan(id)
        
        # TO DO: merge traj plan

    def traffic_state_update(self,car_states):
        """
        Phase I: localize all agents and update path and wpts
        """
        # car_states = self.get_car_state() # Euclidean state
        self.state_update(car_states) # Update PathBuffer, FrenetBuffer, EdgeOccupancy
        """
        Phase II: plan behavior for all agents
        """
        # self.CommandBuffer = []
        # for id in range(self.n_car):
        #     plan = self.plan_car_following(id)
        #     self.CommandBuffer.append(self.excute_plan(plan))
        self.trajectory_update()

    def get_traj_buffer(self):
        return self.TrajBuffer.copy()        


    def pos_pi(self,angle):
        if angle < 0:
            angle += 2*pi
        return angle
    def to_pi(self, angle):
        if angle <= -pi:
            angle += 2*pi
        elif angle > pi:
            angle -= 2*pi
        return angle


WAYPOINT_DISTANCE = 0.5
N_CAR = 20
N_NODE = 42
CAR_PARAM = get_car_param()
CAR_INFO = {"hl":1.775, "hw": 1.0, "amax":3.0, "amin":-3.0, "jerkmax": 10.0} # half length, half width of the car
DT = 0.1
N_LOOP = 10000

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

    plt.clf()
    plt.xlim(0, 60)
    plt.ylim(0, 50)
    plot_topo(node_list,edge_list, ax, if_arrow=False)
    trajbuffer = TM.get_traj_buffer()
    # for traj in trajbuffer:
    #     print("********************* Traj: *****************")
    #     print(traj)
    #     print("*******************************")
    #     plot_traj(traj)
    plot_cars(trajbuffer, car_length, car_width)
    
         


    plt.pause(0.01)

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

    
    

