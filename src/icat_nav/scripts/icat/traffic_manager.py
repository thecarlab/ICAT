
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
        self.look_ahead_dist = 0.2+car_info["hl"]  # For wpts tracking
        self.check_ahead_dist = 1.0 # For car following
        self.safe_clearance = 0.1 + 2*car_info["hl"]  # inter-car center clearance 
        self.constant_speed = 0.2
        self.head_time = 0.5  
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
            new = np.array([self.wpts_dist * n for n in range(len(self.WptsBuffer[id][i]))])+former_s
            Spath = np.concatenate((Spath, new))
            former_s += self.get_edge_length((self.PathBuffer[id][i],self.PathBuffer[id][i+1] )) 
            # print(" Path edge: {}, number of wpts: {}, edge lengdth: {} ".format((self.PathBuffer[id][i],self.PathBuffer[id][i+1]), 
            #         len(self.WptsBuffer[id][i]),self.get_edge_length((self.PathBuffer[id][i],self.PathBuffer[id][i+1])) ))
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

        # print(" car id: ", car_id)
        # print(" car's path: ", path)
        for n in range(len(path)-1):
            edge_points = self.G.get_edge_data(path[n],path[n+1])["waypoints"]
            # print("checking edge: ", (path[n],path[n+1]))
            # print("edge points: ", edge_points)
            closest_index = find_closest_waypoint(state[0], state[1], edge_points)
            closest_point = edge_points[closest_index]
            # print("closet index: ", closest_index)
            # print("closest point: ", closest_point)
            s,d = frenet_transform(state[0], state[1], edge_points, self.wpts_dist)
            print("sd: ",s,d)
            if s < 0 :
                in_edge = (path[n], path[n+1])
                return s,d, in_edge, closest_index, closest_point
            if s >= 0 and s < self.G.get_edge_data(path[n],path[n+1])["weight"]:
                in_edge = (path[n], path[n+1])
                return s,d,in_edge, closest_index, closest_point

        if in_edge == (-1,-1):
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
        ahead_point = ahead_points[clst_index+n_ahead_indices]
        return ahead_in_edge, ahead_index, ahead_point

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
        if ss < 0.0:
            ss = 0
        vs = self.StateBuffer[id]["linear_speed"]       # current speed assume perfectly on the road direction
        a_s =  0.0
        # Calc end state
        leading_id, dist = self.DistBuffer[id]
        if leading_id == -1 or dist >= self.check_ahead_dist:
            print("Too far from the leading car, keep speed")
            se, ve, a_e, minT, maxT, logic = self.calc_velocity_keeping_end_state(id)
        else:
            if dist > self.safe_clearance:

                se, ve, a_e, minT, maxT, logic = self.calc_acc_end_state(id)
            else:
                # print("Calc brake end state")
                se, ve, a_e, minT, maxT, logic = self.calc_brake_end_state(id)
                # print("end stated: ", se)
                # print("vend: ", ve)
                # print("ss: ", ss)
        # assert ss <= se and ss >= 0, "End point is backward!!!"
        # then we plan a trajectory along s coord
        try:
            Ttime, Ts, Tv, Ta, Tj = quintic_1d_plan(ss, vs, a_s, se,ve, a_e,self.car_info["amax"], self.car_info["jerkmax"],self.dt, minT, maxT)  
        except AssertionError:
            # se, ve, a_e, minT, maxT, logic = self.calc_brake_end_state(id)
            # Ttime, Ts, Tv, Ta, Tj = quintic_1d_plan(ss, vs, a_s, se,ve, a_e,self.car_info["amax"], self.car_info["jerkmax"],self.dt, minT, maxT)
            print("Assertion Error!! Trying Brake Traj Planning!!!")
            Ttime,Ts,Tv,Ta,Tj = self.plan_brake_traj(id)  
        Traj= self.calc_traj_from_s(id,Ts,Tv)
        self.TrajBuffer[id] = Traj

    def plan_brake_traj(self, id):
        ss = self.StateBuffer[id]["sd"][0]              # current s coord
        vs = self.StateBuffer[id]["linear_speed"]       # current speed assume perfectly on the road direction   
        a_brake = self.car_info["amax"]
        T = int(vs/a_brake/self.dt)+1
        Ttime, Ts, Tv, Ta, Tj = [0.],[ss],[vs],[a_brake],[0.]
        s = ss
        v = vs
        for i in range(T):
            v = max(0, v-a_brake*self.dt)
            s += v*self.dt   
            Ttime.append(i*self.dt)
            Ts.append(s)
            Tv.append(v)
            Ta.append(a_brake)
            Tj.append(0)
        return Ttime, Ts, Tv, Ta, Tj
                 
    def calc_brake_end_state(self, id):
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
        logic = "brake"
        return se, ve, a_e, minT, maxT, logic

    def calc_velocity_keeping_end_state(self, id):
        se = self.FrenetBuffer[id][0] + self.check_ahead_dist
        ve = self.constant_speed # desired speed
        a_e = 0.0
        minT = self.check_ahead_dist / (1.2 * self.constant_speed) # overspeed no more than 20%
        maxT = self.check_ahead_dist / (self.constant_speed/2)
        logic = "too far"
        return se, ve, a_e, minT, maxT, logic
    
    def calc_acc_end_state(self,id):
        TAU = 0.2
        leading_id, dist = self.DistBuffer[id]
        leading_v = self.StateBuffer[leading_id]["linear_speed"]
        se = self.FrenetBuffer[id][0] + max((dist + TAU* leading_v - self.safe_clearance ),0.)
        ve = 0.0 # ensure a full stop at the tracking end point
        a_e = 0.0 # stop smoothly
        minT = dist /(1.2 * self.constant_speed) # relax a little
        maxT = dist / (self.constant_speed/4)    # enlarge the time area for stable search
        logic = "following"  
        return se, ve, a_e, minT, maxT, logic      
    

    def calc_traj_from_s(self, id, Ts, Tv):
        assert len(Ts) >=2, "Ts length smaller than 2!!"
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
            assert index >= 0, " Wrong localization index"
            # if index <= 0:
            #     index = 0
            ds = Ts[i] - self.Sbuffer[id][index]
            # if ds < 0:
            #     ds = 0
            # print("Car ID: {}, bisect index {}, Ts {}, Sbuffer[id][index] {}, self.LocalWptsBuffer[id]{}".format(id,index, Ts, self.Sbuffer[id][index],self.LocalWptsBuffer[id] ))
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


    # def check_merge_edge(edge):
    #     if edge in self.EdgeOccupancy:
    #         id,s,d = self.EdgeOccupancy[edge][-1]
    #         return id, s
    #     else:
    #         return -1, 0.



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
        # make angle positive
        if angle < 0:
            angle += 2*pi
        return angle
    def to_pi(self, angle):
        # covert angle from -pi to pi
        if angle <= -pi:
            angle += 2*pi
        elif angle > pi:
            angle -= 2*pi
        return angle

    def get_state_buffer(self):
        return self.StateBuffer.copy()
    
    