import numpy as np
import time
import math
import random
import matplotlib.pyplot as plt
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

    
   
    def get_distance( self , first_pose , second_pose):
        return math.sqrt((second_pose[0] - first_pose[0] )**2 + (second_pose[1] - first_pose[1]) **2)
    
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
    
    def not_valid_pose(self, pose): 
        # returns pose that are not valid
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
                        return pose     
                # if  position is not in the map bounds return  is_unknown_valid
                else:
                    if(not self.is_unknown_valid):
                        return pose
        return None



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

    def check_path(self, path):
        step_size = 0.5*self.distance
        print('path',path)
        valid = True
        # TODO: Discretize the positions between 2 waypoints with an step_size = 2*self.distance
        # TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True. 
        for index in range(len(path)-1):

            first_pose  = path[index] # initial path 
            print('first_pose',first_pose)
           
            second_pose = path[index+1] # next path 
            print('second_pose',second_pose)
            # direction vector from first_pose to second_pose
            dir_vector = np.array([second_pose[0] - first_pose[0] , second_pose[1] - first_pose[1]])
            distance = self.get_distance(first_pose , second_pose) # distance from first_pose to second_pose
            
            if distance == 0:
                norm_vect = np.array([0, 0])
            else:
                norm_vect = dir_vector / distance # Compute normal vector b/n two points

            discrtized_seg = np.array([first_pose]) # array of discritized segment 
            current = first_pose

            # while the distance b/n two poses is smaller than min distance 
            while(self.get_distance(second_pose , current) > step_size):          
                current  = current + norm_vect*step_size
                valid = self.is_valid(current)      
                # if one of the discritized poses are not vlid return false    
                if(not valid):
                    return False      
            # Finnally Check the goal point
            valid = self.is_valid(second_pose)

        # If each path is valid return true
        return valid
        
class RRT:
    def  __init__(self,start,goal,state_validity_checker, max_iterations=10000, delta_q=4, p_goal=0.2, dominion=[-10, 10, -10, 10]):
        self.state_validity_checker=state_validity_checker
        self.max_iterations=max_iterations
        self.delta_q=delta_q
        self.p_goal=p_goal
        self.dominion=dominion
        self.start=start
        self.goal_p=goal
        self.nodes=[{'point':np.array(self.start), 'parent': None}]
        
        
    
    def create_random_point(self):
        
        x_min=self.dominion[0]
        x_max=self.dominion[1]
        y_min=self.dominion[2]
        y_max=self.dominion[3]
        if random.random() > self.p_goal:
            random_point = np.array([
            np.random.uniform(x_min, x_max),
            np.random.uniform(y_min, y_max)
        ])
        
        
        else: 
            random_point = self.goal_p
        
        
        
        return random_point
    def find_nearest_node(self, random_point):
        nearest_node = None
        min_distance = np.inf
        for node in self.nodes:
            # print('node',node)
            # print('point',type(node['point']))
            # print('rnd,',type(random_point))
            if node['point'] is not None: 
                # calculate the distance between the current node's point and the given point.
                distance = np.linalg.norm(np.array(node['point'])- np.array(random_point))
                #if the calculated distance is smaller than the current minimum distance then update the nearest node and minimum distance
                if distance < min_distance:
                    nearest_node=node
                    min_distance=distance
               
        return nearest_node['point']
    def determine_new_point(self,nearest_point,random_point):
        print('point',type(nearest_point))
        print('ran',type(random_point))
        direction= np.array(random_point) - np.array(nearest_point)
        distance=np.linalg.norm(direction)
        direction_vector = direction / distance
        if self.delta_q>distance:
            return random_point
        new_point=np.array(nearest_point) + direction_vector* self.delta_q
        
        print('new point',new_point)
        return new_point
        
    def add_new_node(self, new_point):
        if not self.nodes:  # Checks if the list is empty
            parent = None
        else:
            parent = self.nodes[-1]['point']# The last node in the list

        new_node = {'point': new_point, 'parent': parent}
        self.nodes.append(new_node)
        # print('all nodes',self.nodes)

        return new_node

    def find_valid_random_point(self):
        qrand = self.create_random_point()
        if self.state_validity_checker.is_valid(qrand):
            return qrand
        return self.find_valid_random_point()
    
    def check_segment(self,qnew,qnear):
        if self.state_validity_checker.check_path([qnew,qnear]):
            return True
        return False
   

    def retrace_path(self, goal_node):
        path = [goal_node['point']]  # Initialize the path with the goal node's point.
        current_node = goal_node  # Start with the goal node.

        # Trace back to the start node using 'parent' pointers.
        while current_node['parent'] is not None:
            # Find the node whose 'point' matches the 'parent' of the current node.
            parent_node = next((node for node in self.nodes if np.array_equal(node['point'], current_node['parent'])), None)
            
            if parent_node is not None:
                path.append(parent_node['point'])  # Add the parent's point to the path.
                current_node = parent_node  # Update current_node to its parent for the next iteration.
            else:
                break  # If no parent node is found, exit the loop.
         
        return path[::-1]  # Reverse the path to go from start to goal.

    # def smooth_path(self):
    #     counter = 0
    #     max_iterations = 100
    #     self.smoothed_path = [self.goal]  # Start with the goal as the initial point.

    #     # Continue attempting to smooth the path until either we've reached the start node or max iterations.
    #     while counter < max_iterations:
    #         path_modified = False  # Track if any changes were made during the iteration.

    #         for i in range(len(self.path) - 2, -1, -1):  # Iterate backwards through the path, skipping the last node (goal).
    #             node = self.path[i]
    #             last_in_smoothed = self.smoothed_path[-1]  # Last node currently in the smoothed path.

    #             # Check if a direct path is possible from the current node to the last node in the smoothed path.
    #             if self.svc.check_path([[node.x, node.y], [last_in_smoothed.x, last_in_smoothed.y]]):
    #                 self.smoothed_path.append(node)  # If so, add this node to the smoothed path.
    #                 path_modified = True
    #                 break  # Break after modifying the path to restart the smoothing from this new point.

    #         # If no modifications were made in this iteration or we've added the start node, stop smoothing.
    #         if not path_modified or self.smoothed_path[-1] == self.start:
    #             break

    #         counter += 1

    # # If the smoothing process completed due to reaching max iterations without fully simplifying the path.
    #     if counter >= max_iterations:
    #         print("Max iteration reached, reverting to original path.")
    #         self.smoothed_path = self.path.copy()

    #     # Ensure the path is in the correct order, from start to goal.
    #     self.smoothed_path.reverse()
    #     # Convert the smoothed path nodes to a list of coordinate tuples.
    #     path = [(n.x, n.y) for n in self.smoothed_path]

    #     return path

    def compute_path(self,qgoal):
        for _ in range(self.max_iterations):
            qrand=self.find_valid_random_point()
            qnear=self.find_nearest_node(qrand)
            qnew=self.determine_new_point(qnear,qrand)
            check_segment_new_near=self.check_segment(qnew,qnear)
           
            if check_segment_new_near:
                new_node=self.add_new_node(qnew)
                print('new_node',new_node)
                if np.linalg.norm(new_node['point'] - qgoal) <= self.delta_q and self.check_segment(new_node['point'], qgoal):
                     goal_node = self.add_new_node(qgoal)
                     print('goal_node',goal_node)
                     print(self.nodes)
                     total_path=self.retrace_path(goal_node)
                     print('t',total_path)
                     if self.state_validity_checker.check_path(total_path):
                        return total_path
        return []
                     
                
              
                
           


def compute_path(start_p, goal_p, state_validity_checker, bounds, max_time=1):
    # Create an instance of RRT with the given parameters
    rrt = RRT(start_p,goal_p, state_validity_checker=state_validity_checker, max_iterations=10000, delta_q=4, p_goal=0.2, dominion=bounds)

  
    path = rrt.compute_path(goal_p)
        
       
    return path  # Return None if no path is found within the maximum time

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
    
    #propotional control
    # v = Kv * distance_error 
    # w = Kw * orientation_error
    # If the orientation error is significant, prioritize rotating towards the goal
    # if abs(orientation_error) > np.pi /6:
    #     v = 0  # Stop linear movement to focus on rotation
    #     w = Kw * orientation_error
    # else:
    #     v = Kv * distance_error
    #     w = Kw * orientation_error  # Continue adjusting orientation while moving
    
    # if allow_reverse and abs(orientation_error) > reverse_threshold:
    #     # If the goal is behind and reverse is allowed, adjust to move backward
    #     adjusted_angle_diff = wrap_angle(theta_goal - (theta_current + np.pi)) # Reverse direction
    #     v = -Kv * distance_error  # Move backward
    #     w = Kw * adjusted_angle_diff
    # else:
    #     # Normal forward movement
    #     v = Kv * distance_error if abs(orientation_error) < np.pi / 6 else 0
    #     w = Kw * orientation_error
    # Determine movement direction (forward or backward) based on orientation error
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

 
 