import numpy as np
import random
import math
from dubins_path import draw_tangent_line, draw_arc_circle, segment_crosses_circle, circle_crosses_circle
from obstacles import segment_intersects_circle
from obstacles import circles_intersect
from config import (
    FLIGHT_SPEED, B_MIN, B_MAX, ALPHA_MAX, RHO, CD0, M, G,
    TMAX, GAMMA_ACT, R_MIN_THRESHOLD, CL, V_MAX, 
    WING_SURFACE_MIN, WING_SURFACE_MAX, VARIABLE_SPEED
)

class Individual:

    def __init__(self, x_init, x_goal, obs, tangency = None, vp = None):
 
        ### INITIALIZATIONS       
        self.x_init =   x_init
        self.x_goal =   x_goal
        self.obs    =   obs.copy()
        self.n_obs  =   len(obs)
        self.tangency = [9]*self.n_obs
        # Unified arrays: [arc0, tan0, arc1, tan1, ..., arcN-1, tanN-1, arcN]
        self.length = [0]*self.n_obs
        self.wingspan = ["NaN"]*(2*self.n_obs)
        self.v = ["NaN"]*(2*self.n_obs)
        self.r_min = ["NaN"]*(2*self.n_obs)
        self.path, self.arc_list, self.tan_list = [[],[]],[],[]
        self.arc_path = [[], []]  # [x_arc_points, y_arc_points]
        self.tan_path = [[], []]  # [x_tan_points, y_tan_points]
        self.obsCrossed = []
        self.flight_time = 999999
        self.path_wingspan = []
        # Initialize parameters per segment
        if vp is not None: 
            self.v =        vp[0]
            self.wingspan = vp[1]
            self.r_min =    vp[2]
        if  vp is None:
            self.reset_vehiclePars()        

        # Initialize how we approach each obstacle
        if tangency is not None: 
            self.tangency = tangency
        if tangency is None:
            self.reset_tangencies()
        ### PATH COMPUTATION
        self.find_path()
        
### FITNESS EVALUATION
        self.score = self.fitness()

    def reset_tangencies(self):
        "Randomize tangency approach (RT, LT, NT) for all obstacles"
        for i in range(self.n_obs):
            self.randomize_tangencies(i)

    def randomize_tangencies(self, i, turn = None):
        "Randomize tangency approach (RT, LT, NT) for obstacle i"
        if (i == 0) or (i==self.n_obs-1) or turn:
            mode = ["RT", "LT"]
        else:
            mode = ["RT", "LT", "NT"]
        self.tangency[i] = random.choice(mode)

    def read_tangencies(self,i):
        """
        Translate two sequential obstacles tangencies (RT,LT,NT) into a 
        dubins path command (RSR, RSL,LSR,LSL)
        """
        next = i+1
        while((self.tangency[next]=="NT") and next<self.n_obs) :
            next+=1

        R1 = max(self.r_min[2*i], self.obs[i]['radius'] + self.wingspan[2*i])
        R2 = max(self.r_min[2*next], self.obs[next]['radius'] + self.wingspan[2*next])

        #If the obstacles are too close, change tangency to RSR or LSL to avoid collision      
        if circle_crosses_circle(self.obs[i]['x'], self.obs[i]['y'], R1, 
                                 self.obs[next]['x'], self.obs[next]['y'], R2):
            self.tangency[next] = self.tangency[i]

        if self.tangency[i]== "RT":
            if self.tangency[next]=="RT":
                mode = "RSR"
            if self.tangency[next]=="LT":
                mode = "RSL"

        if self.tangency[i]=="LT":
            if self.tangency[next]== "LT":
                mode =  "LSL"
            if self.tangency[next]=="RT":
                mode = "LSR"

        return mode, next

#   COMPUTE SCORE
    def fitness(self):
        "Evaluate the fitness of this chromosome"
        # Calculate segment-wise time using v[i]
        x, y = self.path[0], self.path[1]
        diffs = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
        speeds = np.array([v if isinstance(v, (int, float, np.floating)) and v > 0 else FLIGHT_SPEED for v in self.v])
        # If speeds length doesn't match points, pad or trim as needed
        if len(speeds) < len(x):
            speeds = np.pad(speeds, (0, len(x)-len(speeds)), 'edge')
        elif len(speeds) > len(x):
            speeds = speeds[:len(x)]
        # For each segment, use v0 (start), v1 (end)
        segment_times = []
        for i, L in enumerate(diffs):
            v0 = speeds[i]
            v1 = speeds[i+1]
            # L = t*(v0+v1)/2 => t = 2*L/(v0+v1)
            v_sum = v0 + v1
            if v_sum > 0:
                t = 2*L/v_sum
            else:
                t = L/FLIGHT_SPEED
            segment_times.append(t)
        self.flight_time = np.sum(segment_times)
        self.tangency_penalty = sum(1 for t in self.tangency if t != "NT")
        
        return  10000000*(len(self.obsCrossed)) + 200*self.flight_time +  10*self.tangency_penalty
    

    ## FIND PATH
    def find_path(self):
        "Appends together the path of each segment, as a serie of points [x,y]"
        x, y, current_point = [], [], self.x_init
        arc_x, arc_y = [], []
        tan_x, tan_y = [], []
        i = 0

        while (i < self.n_obs - 1):
            # Find the path from one obstacle to the next, as a series of points [x,y]
            next, path_x, path_y, arc_seg_x, arc_seg_y, tan_seg_x, tan_seg_y, current_point = self.find_segment(i, current_point)
            
            # Full path
            x, y = np.append(x, path_x), np.append(y, path_y)
            # Arc path
            arc_x = np.append(arc_x, arc_seg_x)
            arc_y = np.append(arc_y, arc_seg_y)
            # Tangent path
            tan_x = np.append(tan_x, tan_seg_x)
            tan_y = np.append(tan_y, tan_seg_y)
            #p1 and p2 are the start and end points of the tangent segment
            p1 = (tan_seg_x[0], tan_seg_y[0])
            p2 = (tan_seg_x[1], tan_seg_y[1])


            # For each obstacle, check if the tangent segment or the arc segment crosses any other obstacle (except the one we are leaving)
            is_crossing = False
            arc_data = self.arc_list[-1] if self.arc_list else None

            for j in range(len(self.obs)-1):
                if j != i:
                    circle = {
                        'x': self.obs[j]['x'],
                        'y': self.obs[j]['y'],
                        'radius': self.obs[j]['radius'] + self.wingspan[2*i]/2
                    }
                    # Check tangent segment crossing
                    if segment_crosses_circle(p1, p2, circle):
                        is_crossing = True

                    # Improved arc-circle collision detection
                    if arc_data is not None:
                        arc_center = (arc_data[0]['x'], arc_data[0]['y'])
                        arc_radius = arc_data[0]['radius'] + self.wingspan[2*i]/2
                        angle_start = arc_data[1]
                        angle_end = arc_data[2]
                        other_center = (self.obs[j]['x'], self.obs[j]['y'])
                        other_radius = self.obs[j]['radius'] + self.wingspan[2*i]/2

                        from dubins_path import arc_circle_crosses_circle
                        if arc_circle_crosses_circle(arc_center, arc_radius, angle_start, angle_end, other_center, other_radius):
                            is_crossing = True

                    if is_crossing:
                        if j not in self.obsCrossed:
                            self.obsCrossed.append(j)
                    is_crossing = False
            i = next

        self.path = [x, y]
        self.arc_path = [arc_x, arc_y]
        self.tan_path = [tan_x, tan_y]
    
    def find_segment(self, i, current_point):
        """
            Finds the path from one obstacle to the next, as a series of points [x,y]
        """

        mode, next = self.read_tangencies(i)
        R1 = max(self.r_min[2*i], self.obs[i]['radius'] + self.wingspan[2*i]/2)
        R2 = max(self.r_min[2*next], self.obs[next]['radius'] + self.wingspan[2*next]/2)
        vehiclePars = [R1, R2, self.wingspan[2*i+1], self.wingspan[2*next+1]]
        # Add a tangent line between current circle and next circle, from p1 to p2
        p1, p2, b_1 = draw_tangent_line(self.obs[i], self.obs[next], mode, vehiclePars)

        vehiclePars = [R1, R2, self.wingspan[2*i], self.wingspan[2*next]]
        # Add an arc circle on the starting circle, from current point to takeoff point p1
        path_arc, arc_data = draw_arc_circle(self.obs[i], p1, current_point, mode, vehiclePars)
        
        # The path is the concatenation of the arc and the tangent line
        path_x = np.concatenate((path_arc[0], np.array([p1[0], p2[0]])))
        path_y = np.concatenate((path_arc[1], np.array([p1[1], p2[1]])))
        # For visualization, alternate arc and tangent wingspans for each segment
        # path_x: [arc_points..., p1, p2] (arc then tangent)
        n_arc = len(path_arc[0])

        self.path_wingspan.extend([self.wingspan[2*i]]*n_arc)
        self.path_wingspan.extend([self.wingspan[2*i+1]]*2)
        # Optionally, you could store v/r_min for each segment for further use

        # Arc segment 
        arc_seg_x = path_arc[0]
        arc_seg_y = path_arc[1]
        self.arc_list.append(arc_data)

        # Tangent segment (just p1 to p2)
        tan_seg_x = np.array([p1[0], p2[0]])
        tan_seg_y = np.array([p1[1], p2[1]])
        self.tan_list.append([p1, p2, b_1])

        return next, path_x, path_y, arc_seg_x, arc_seg_y, tan_seg_x, tan_seg_y, p2


    



#INITIALIZATION
    def compute_vehiclePars(self, wingspan, idx):

        wingSurface =  0.0531 + 0.0012*np.exp(4.6945*wingspan)
        flightSpeed    = np.sqrt(2*M*G/RHO/wingSurface/CL)
        Lift_arc       = 0.5*RHO*wingSurface*CL*flightSpeed**2
        cD_arc         = CD0 + (wingSurface*CL**2/(np.pi* wingspan **2))
        Dmax_arc       = 0.5 * RHO*wingSurface*cD_arc*flightSpeed**2
        phiMax_arc     = np.arccos((M*G*np.cos(GAMMA_ACT))/(Lift_arc+ TMAX*np.sin(ALPHA_MAX)))

        self.v[idx] = flightSpeed
        self.r_min[idx]   = pow( flightSpeed,2 ) / (G * np.tan( phiMax_arc ) )
        self.wingspan[idx] = wingspan

    def randomize_vehiclePars(self, idx):
        wingspan = random.uniform(B_MIN, B_MAX)
        self.compute_vehiclePars(wingspan, idx)


    def reset_vehiclePars(self):
        for i in range(2*self.n_obs):
            self.randomize_vehiclePars(i)
        self.r_min[0] = 0
        self.r_min[2*self.n_obs-1] = 0


