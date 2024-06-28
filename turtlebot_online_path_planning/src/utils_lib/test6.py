import numpy as np
import time
import matplotlib.pyplot as plt
import copy
import math
def wrap_angle(angle):
    #corrects an angle to be within the range of [-pi, pi]
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.3, is_unknown_valid=True):
        self.map = None
        self.resolution = None
        self.origin = None
        self.there_is_map = False
        self.distance = distance
        self.is_unknown_valid = is_unknown_valid
    
    # Set occupancy map, its resolution and origin
    def set(self, data, resolution, origin):
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
        self.height = data.shape[0]
        self.width = data.shape[1]
    # Given a pose, returns true if the pose is not in collision and false otherwise
    
    def is_valid(self, pose): 
        # TODO: convert world robot position to map coordinates using method __position_to_map__
        # TODO: check occupancy of the vicinity of a robot position (indicated by self.distance atribute). 
        # Return True if free, False if occupied and self.is_unknown_valid if unknown. 
        # If checked position is outside the map bounds consider it as unknown.
        m = self.__position_to_map__(pose)
        grid_distance = int(self.distance/self.resolution)
        # add 6 points upper and to lower limit 
        lower_x , lower_y = m[0] - grid_distance , m[1] - grid_distance  
        for lx in range(0,2*grid_distance):
            for ly in range(0,2*grid_distance):
                pose = lower_x + lx, lower_y + ly              
                # if one of the position is not free return False  , stop the loop 
                if(self.is_onmap(pose)):                           
                    if(not self.is_free(pose)): 
                        return False     
                # if  position is not in the map bounds return  is_unknown_valid
                else:
                    if(not self.is_unknown_valid):
                        return False
        return True
    def __position_to_map__(self, p):
        print('pose',p)
        
        x , y  =  p # get x and y positions 
        m_x    =  (x - self.origin[0])/self.resolution  # x cell cordinate 
        m_y    =  (y - self.origin[1])/self.resolution  # y cell cordinate 

        return [round(m_x), round(m_y)]
    # Given a path, returns true if the path is not in collision and false otherwise
    # In RRT class
    
    # Define RRT class (you can take code from Autonopmous Systems course!)
    def __map_to_position__(self, m):
        x ,y = m  
        p_x  = self.origin[0] + x * self.resolution  
        p_y  = self.origin[1] + y * self.resolution 

        return [p_x, p_y]
    def is_onmap(self,pose):    
        # checks if a given pose in grid is on the map 
        # if is is on the map return Truem else return is False 
        if( 0<=pose[0]< self.height and 0<= pose[1] < self.width):
            return True        
        else:
            return False
    def is_free(self, pose): 
        # if is is free return True , which means  
        if self.map[pose[0],pose[1]] == 0 :
            return True, 
        #if it is unkown return opposite of is unkown valid 
        elif self.map[pose[0],pose [1]] == -1 : # return opposite of is unkown valid 
            return  self.is_unknown_valid
        # return false , if is is obstacle 
        return False   
    # Given a path, returns true if the path is not in collision and false otherwise
    def calculate_distance(self, first_pose , second_pose):
        """
        Calculate Euclidean distance between two points.
        """
        return math.sqrt((second_pose[0] - first_pose[0] )**2 + (second_pose[1] - first_pose[1]) **2)
    

    def check_path(self, path):
        step_size = 0.5*self.distance
        valid = True
        # TODO: Discretize the positions between 2 waypoints with an step_size = 2*self.distance
        # TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True. 
        for index in range(len(path)-1):

            first_pose  = path[index] # initial path 
            second_pose = path[index+1] # next path 
            # direction vector from first_pose to second_pose
            dir_vector = np.array([second_pose[0] - first_pose[0] , second_pose[1] - first_pose[1]])
            distance = self.calculate_distance(first_pose , second_pose) # distance from first_pose to second_pose
            
            if distance == 0:
                norm_vect = np.array([0, 0])
            else:
                norm_vect = dir_vector / distance # Compute normal vector b/n two points

            discrtized_seg = np.array([first_pose]) # array of discritized segment 
            current = first_pose

            # while the distance b/n two poses is smaller than min distance 
            while(self.calculate_distance(second_pose , current) > step_size):          
                current  = current + norm_vect*step_size
                valid = self.is_valid(current)      
                # if one of the discritized poses are not vlid return false    
                if(not valid):
                    return False      
            # Finnally Check the goal point
            valid = self.is_valid(second_pose)

        # If each path is valid return true
        return valid
    # Transform position with respect to the map origin to cell coordinates
    def _position_to_map_(self, p):
        #check if map has been initialized, if not raise an error
        
        if not self.there_is_map:
            raise ValueError("Map has not been set yet.")
        
        #calculate the coordinates on the map by adjusting the position
        #the floor of the input, element-wise, gives integer value
        coords = np.floor((p - self.origin) / self.resolution).astype(int)
        
        #check if the coordinates are within the map boundaries
        
        if np.all(coords >= 0) and np.all(coords < np.array(self.map.shape)):
            return coords
        #return None if the coordinates are outside the map's boundaries.
        else:
            return None
            
    
    
    
    ###################   RRT  ################################
import numpy as np 
from matplotlib import pyplot as plt 
from PIL import Image 
from math import sqrt
import random

class Node:
    def __init__(self, x, y):
        self.x = x  # x-position
        self.y = y  # y-position
        self.id = 0  # vertex id
        self.f_score = float('inf')  # initialize as infinity
        self.g_score = float('inf')  # initialize as infinity
        self.parent = None

    def calculate_distance(self, target):
        """Calculate the Euclidean distance between the current node and the target node."""
        return sqrt((self.x - target.x) ** 2 + (self.y - target.y) ** 2)

    def find_nearest_node(self, nodes):
        """Find the nearest node from the given nodes."""
        nearest_node, min_distance = min(((node, self.calculate_distance(node)) for node in nodes),
                                         key=lambda x: x[1], default=(None, float('inf')))
        return nearest_node

    def find_nodes_within_radius(self, nodes, radius):
        """Find all nodes within the given radius."""
        return [node for node in nodes if self.calculate_distance(node) <= radius]

    def __str__(self):
        return str(self.id)

# The code is optimized for readability, efficiency, and adherence to Python conventions.
# Functionality is preserved with improved method names and reduced redundancy.

# Define RRT class (you can take code from Autonopmous Systems course!)
class RRTStar:
    def __init__(self, svc, k, q, p, dominion=[-10, 10, -10, 10], max_time=0.1):
        self.svc = svc
        self.k = k
        self.q = q
        self.p = p
        self.dominion = dominion
        self.max_time = max_time
        self.vertices = []
        self.edges = []
        self.node_counter = 1
        self.path = []
        self.smoothed_path = []
        self.radius = 2  # Radius for RRT* search method

    def cost_optimal_parent(self, qnew, current_parent):
        nodes_within_radius = qnew.find_nodes_within_radius(self.vertices, self.radius)
        best_cost = current_parent.g_score + qnew.calculate_distance(current_parent)
        best_parent = current_parent
        
        for node in nodes_within_radius:
            new_node_cost = node.g_score + qnew.calculate_distance(node)
            if new_node_cost < best_cost and self.svc.check_path([[node.x, node.y], [qnew.x, qnew.y]]):
                best_parent = node
                best_cost = new_node_cost

        return best_parent

    def rewire(self, qnew):
        nodes_within_radius = qnew.find_nodes_within_radius(self.vertices, self.radius)
        for node in nodes_within_radius:
            new_node_cost = qnew.g_score + qnew.calculate_distance(node)
            
            if new_node_cost < node.g_score and self.svc.check_path([[node.x, node.y], [qnew.x, qnew.y]]):
                node.parent = qnew
                node.g_score = new_node_cost
                # Assuming there's a method to update f_score based on a new parent
                node.f_score = node.g_score + node.calculate_distance(self.goal)

    def rand_config(self):
        prob = random.random()
        x = random.uniform(self.dominion[0], self.dominion[1])
        y = random.uniform(self.dominion[2], self.dominion[3])
        qrand = Node(x, y)
        
        if prob < self.p:
            qrand = self.goal

        return qrand

    def near_vertices(self, qrand):
        return qrand.find_nearest_node(self.vertices)

    def new_config(self, qnear, qrand):
        dir_vector = np.array([qrand.x - qnear.x, qrand.y - qnear.y])
        length = qnear.calculate_distance(qrand)
        norm_vector = dir_vector / length
        
        if self.q > length:
            return qrand
        
        qnew_position = np.array([qnear.x, qnear.y]) + norm_vector * self.q
        return Node(qnew_position[0], qnew_position[1])

    def reconstruct_path(self):
        current = self.goal
        self.path = [current]
        while current != self.start:
            current = current.parent
            self.path.append(current)
        
        self.path.reverse()
        return [(n.x, n.y) for n in self.path]

    def smooth_path(self):
        counter = 0
        max_iterations = 100
        self.smoothed_path = [self.goal]
        while True:
            for node in self.path:
                next_path = self.smoothed_path[len(self.smoothed_path)-1]
                if(self.svc.check_path([[node.x , node.y],[next_path.x, next_path.y]])):
    
                    self.smoothed_path.append(node)
                    break
            if self.smoothed_path[len(self.smoothed_path)-1] == self.start:
                break
            counter +=1
        if(counter >= max_iterations):
            print("max iteration reached")
            self.smoothed_path = self.path

        self.smoothed_path.reverse()
        path =[(n.x,n.y) for n in self.smoothed_path]
        
        return path

    def get_tree(self):
        return [[[edge[0].x, edge[0].y], [edge[1].x, edge[1].y]] for edge in self.edges]

    def compute_path(self, start, goal):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.vertices.append(self.start)
        self.start.g_score = 0
        self.start.f_score = self.start.calculate_distance(self.goal)

        for _ in range(self.k):
            qrand = self.rand_config()
            while not self.svc.is_valid([qrand.x, qrand.y]):
                qrand = self.rand_config()
            
            qnear = self.near_vertices(qrand)
            qnew = self.new_config(qnear, qrand)
            
            if self.svc.check_path([[qnear.x, qnear.y], [qnew.x, qnew.y]]):
                qnear = self.cost_optimal_parent(qnew, qnear)
                self.vertices.append(qnew)
                self.edges.append((qnear, qnew))
                qnew.parent = qnear
                qnew.id = self.node_counter
                qnew.g_score = qnear.g_score + qnew.calculate_distance(qnear)
                qnew.f_score = qnew.g_score + qnew.calculate_distance(self.goal)

                self.rewire(qnew)
                
                if qnew == self.goal:
                    return self.reconstruct_path(), self.get_tree()
                
                self.node_counter += 1
        
        return [], self.get_tree()

# Note: This refactored class assumes the existence of the Node class and its methods as previously optimized.
# Further modifications may be necessary depending on the specific implementations of svc and smoothing algorithm.

# Planner: This function has to plan a path from start_p to goal_p. To check if a position is valid the 
# StateValidityChecker class has to be used. The planning dominion must be specified as well as the maximum planning time.
# The planner returns a path that is a list of poses ([x, y]).
    # TODO: Plan a path from start_p to goal_p inside bounds using the RRT and the 
    # StateValidityChecker Objects previously defined.
    # TODO: if solved, return a list with the [x, y] points in the solution path.
    # example: [[x1, y1], [x2, y2], ...]
    # TODO: Ensure that the path brings the robot to the goal (with a small tolerance)!
    
    
def compute_path(start_p, goal_p, svc, bounds , max_time=1.0):

    rrt = RRTStar(svc ,10000 ,3, 0.2 , bounds, max_time)
    path  , tree_list = rrt.compute_path(start_p, goal_p)
    # TODO: Plan a path from start_p to goal_p inside bounds using the RRT and the 
    # StateValidityChecker Objec    ts previously defined.
    # TODO: if solved, return a list with the [x, y] points in the solution path.
    # example: [[x1, y1], [x2, y2], ...]
    # TODO: Ensure that the path brings the robot to the goal (with a small tolerance)!
    return path  , tree_list

              
    
   


# Controller: Given the current position and the goal position, this function computes the desired 
# lineal velocity and angular velocity to be applied in order to reah the goal.
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    
    # TODO: Use a proportional controller which sets a velocity command to move from current position to goal (u = Ke)
    # To avoid strange curves, first correct the orientation and then the distance. 
    # Hint: use wrap_angle function to maintain yaw in [-pi, pi]
    # This function should return only  linear velocity (v) and angular velocity (w)
    # extract current and goal positions and orientations

    x_current, y_current, theta_current = current
    x_goal, y_goal= goal
    
    
    # maximum linear velocity control action                   
    v_max = 0.15
    # maximum angular velocity control action               
    w_max = 0.3 
    
    
    #calculate the position error
    dx=x_goal-x_current
    dy = y_goal - y_current
    distance_error = np.sqrt(dx**2 + dy**2)
    
    
    #goal orientation from current position to goal position
    theta_goal = np.arctan2(dy, dx)
    
    # calculate orientation error
    orientation_error = wrap_angle(theta_goal - theta_current)
    
    if abs(orientation_error) > np.pi / 2:
        # If the goal is behind, consider rotating first or moving backward
        orientation_error = wrap_angle(orientation_error - np.pi)  # Adjust for reverse
        distance_error *= -1  # Move backward
        
    # Apply proportional control
    v = Kv * distance_error
    w = Kw * orientation_error
    
    #  clip velocities to their maximum speed
    v = np.clip(v, -v_max, v_max)   
    w = np.clip(w, -w_max, w_max)
    return v, w



