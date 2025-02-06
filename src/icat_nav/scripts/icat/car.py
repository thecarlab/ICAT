import numpy as np
from math import sqrt, sin, cos, pi,ceil
from nav_gym.obj.robot.robot import CarRobot
class CarParam:
    """
    ICAT car parameter:
    Wheelbase = 23.5 cm
    Frontsus  =  8.0 cm
    Rearsus   =  4.0 cm
    Halfwidth = 10.0 cm
    Lidar     = 13.0 cm
    """
    def __init__(self, shape = np.array([2.35,0.8,0.4,1.0,1.3]), type = "diff", round_r = 1.5):
        # if type == "normal": # The params are based on our own robot ZebraT
        # wheelbase, front, rear suspension, half width, lidar to center(defaut mount on the front and midline)
        # self.shape = np.array([0.53,0.25,0.25,0.35,0.65])
        self.shape = shape
        self.type = type
        assert self.type in ["car", "diff", "round"], "Wrong driving type!"
        self.safe_dilation = 0.1 # dilate the counter with the value
        # limits 2X2 box ([v1_max, v2_max],
        #                 [v1_min, v2_min]) 
        # where v1 is linear speed, v2 is actually steering angle instead of angular speed in differential robots
        # self.v_limits = np.array([[2.,0.6 ],
        #                          [-1., -0.6]])
        # self.a_limits = np.array([[0.8, 1.0],
        #                          [-1., -1.0]])
        self.v_limits = np.array([[1.0, 1.0 ],
                                    [-0.6, -0.6]])
        self.a_limits = np.array([[0.8, 1.0],
                                    [-1., -1.0]])
        if type == "diff" or type == "round":
            self.v_limits = np.array([[5., 1.0 ],
                                        [0., -1.0]])
            self.a_limits = np.array([[3., 1.5],
                                        [-3., -1.5]])
        # Lidar param
        self.fan_range = np.array([0, 2*pi])
        self.min_range = 0.1
        self.max_range = 10.0
        self.ray_num = 64
        self.angle_reso = 0.098
        # Goal parameters
        self.look_ahead_dist = 2.0 # look-ahead distance / local goal radius
        # Value map parameters (this is for lidar rays generation)
        self.value_base = 0.9900
        self.dv = 0.0001
        # Map parameters
        self.world_reso = 0.01
        # Other
        self.achieve_tolerance = 0.3

        """
        The following params are imposed from shape.
        """
        if self.type == "round":
            self.geo_r = round_r
        else:
            self.geo_r = self.calc_geo_r()                  # Geometry circle radius
        self.disk_r = self.calc_disk_r()                # Collision circle radius
        self.disk_num = self.calc_disk_num()            # Collision circle number
        self.disk_centers = self.calc_disk_centers()    # Centers of collision circles
        # self.vertices contains array([four vertices + lidar center + geometry center])
        self.vertices = self.calc_vertices() # calc vertices and geometry center relative vector to vehicle center, then do tfs
        self.safe_vertices = self.calc_safe_vertices()

    def calc_safe_vertices(self):
        wheel_base = self.shape[0]
        front_sus = self.shape[1]
        rear_sus = self.shape[2]
        half_width = self.shape[3]
        d = 2*self.safe_dilation
        d_l = self.shape[4]
        rl = np.array([-rear_sus-d-d_l, half_width+d]) # rear left vertice
        rr = np.array([-rear_sus-d-d_l, -half_width-d])
        fl = np.array([wheel_base+front_sus+d-d_l, half_width+d ])
        fr = np.array([wheel_base+front_sus+d-d_l, -half_width-d])
        gc = np.array([(wheel_base+rear_sus+front_sus)/2-rear_sus, 0]) # Geometry center
        if self.type == "car":
            return np.array([rl,rr,fr,fl])
        elif self.type == "diff" or "round":
            # print("ccccc",fl, " ", gc)
            return np.array([rl,rr,fr,fl])-gc


    def calc_vertices(self):
        wheel_base = self.shape[0]
        front_sus = self.shape[1]
        rear_sus = self.shape[2]
        half_width = self.shape[3]
        rl = np.array([-rear_sus, half_width]) # rear left vertice
        rr = np.array([-rear_sus, -half_width])
        fl = np.array([wheel_base+front_sus, half_width ])
        fr = np.array([wheel_base+front_sus, -half_width])
        lc = np.array([self.shape[4], 0.]) # Lidar center 
        gc = np.array([(wheel_base+rear_sus+front_sus)/2-rear_sus, 0]) # Geometry center
        if self.type == "car":
            return np.array([rl,rr,fr,fl,lc,gc])
        elif self.type == "diff" or "round":
            return np.array([rl,rr,fr,fl,lc,gc])-gc




    def calc_disk_r(self):
        half_width = self.shape[3]
        return np.sqrt(half_width**2/2)
    
    def calc_geo_r(self):
        wheel_base, front_sus, rear_sus, half_width = self.shape[0:4]
        half_len = (wheel_base+front_sus+rear_sus)/2
        print("Geo Radius: ", sqrt(half_len**2 + half_width**2) )
        return sqrt(half_len**2 + half_width**2) 

    def calc_disk_num(self):
        wheel_base = self.shape[0]
        front_sus = self.shape[1]
        rear_sus = self.shape[2]
        half_width = self.shape[3]
        num = ceil(wheel_base+front_sus+rear_sus)/ (2* half_width )
        return num.astype(np.int32)
    
    def calc_disk_centers(self):
        # First calc front and rear disk
        wheel_base = self.shape[0]
        front_sus = self.shape[1]
        rear_sus = self.shape[2]
        half_width = self.shape[3]
        centers=[]
        rear_center = np.array([half_width, 0.])
        front_center = np.array([wheel_base+rear_sus+front_sus-half_width, 0.])
        centers.append(rear_center)
        dx = (front_center[0] - rear_center[0])/(self.disk_num-1)
        for i in range(self.disk_num-2):
            centers.append(np.array([rear_center[0]+dx*(i+1),0.]))
        centers.append(front_center)
        return np.array(centers)
    

def build_car(i, CAR_PARAM, coord):
    x,y,yaw = coord
    car = CarRobot(id=i, param= CAR_PARAM, initial_state=np.array([x,y,yaw,0.,0.]), dt = 0.1)
    return car

def get_car_param():
    param = CarParam()
    return param