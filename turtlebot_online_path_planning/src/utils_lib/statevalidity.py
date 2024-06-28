import numpy as np
import time
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
    
    # Given a pose, returns true if the pose is not in collision and false otherwise
    
    def is_valid(self, pose):
        if not self.there_is_map:
            raise ValueError("Map has not been set yet.")

        map_coords = self._position_to_map_(pose)  # Convert pose to grid coordinates
        if map_coords is None:
            return self.is_unknown_valid
        
        y, x = map_coords
        radius_cells = int(self.distance / self.resolution)  # Calculate the number of cells to check around the pose
        
        # Check the cells around the pose within the specified distance
        for i in range(y - radius_cells, y + radius_cells + 1):
            for j in range(x - radius_cells, x + radius_cells + 1):
                if i < 0 or j < 0 or i >= self.map.shape[0] or j >= self.map.shape[1]:
                    if not self.is_unknown_valid:
                        return False
                    continue
                if self.map[i][j] != 0:  # 0 for free space, otherwise occupied
                    return False
        return True
    

    # Given a path, returns true if the path is not in collision and false otherwise
    def check_path(self, path):
        if not self.there_is_map:
            raise ValueError("Map has not been set yet.")

        for i in range(len(path) - 1):
            start = self._position_to_map_(path[i])
            end = self._position_to_map_(path[i + 1])

            if start is None or end is None:
                return False

            direction = np.array(end) - np.array(start)
            steps = max(abs(direction)) // (self.distance/2)

            for s in range(int(steps) + 1):
                intermediate_point = start + direction * (s / steps)
                intermediate_point = np.round(intermediate_point).astype(int)
                map_coords = self._position_to_map_(intermediate_point)
                if map_coords is None or self.map[map_coords[0], map_coords[1]]!=0:
                    return False

        return True

    # Transform position with respect to the map origin to cell coordinates
    def _position_to_map_(self, p):
        #check if map has been initialized, if not raise an error
        
        if not self.there_is_map:
            raise ValueError("Map has not been set yet.")
        # #convert input position to numpy array 
        # p = np.array(p)
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

# Define RRT class (you can take code from Autonopmous Systems course!)
class RRT:
    def  __init__(self, state_validity_checker, max_iterations=10000, delta_q=4, p_goal=0.2, dominion=[-10, 10, -10, 10]):
        # define constructor ...
        self.state_validity_checker=state_validity_checker #checks if a given state is valid or not
        self.max_iterations=max_iterations #maximum number of iterations to run the RRT algorithm
        self.delta_q=delta_q #step size
        self.p_goal=p_goal #goal point probability
        self.dominion=dominion #search space dominion, defined as [x_min, x_max, y_min, y_max]
        
        #tree is initially empty or has a dummy node
        self.nodes = []

        
    #create a random point within the search space dominion.
    def create_random_point(self):
      
        x_min, x_max, y_min, y_max = np.array([-3, 3, -2, 2])
        random_point = np.array([
            np.random.uniform(x_min, x_max),
            np.random.uniform(y_min, y_max)
        ])
        
        return random_point
            
    
    #find the nearest node to the given point
    def find_nearest_node(self,point):
        
        #initialize nearest node and minimum distance
        nearest_node=None
        min_distance=np.inf
        for node in self.nodes:
            if node['point'] is not None: 
                # calculate the distance between the current node's point and the given point.
                distance = np.linalg.norm(np.array(node['point'][:2])- np.array(point[:2]))
                #if the calculated distance is smaller than the current minimum distance then update the nearest node and minimum distance
                if distance < min_distance:
                    nearest_node=node
                    min_distance=distance
        return nearest_node
    
    #add nodes to the tree
    def add_node(self, point, parent):
         #create a dictionary to represent the new node with its point and parent
        new_node = {'point': point, 'parent': parent}
        # # append the new node to the list of nodes
        self.nodes.append(new_node)
        print('nodes',self.nodes)
        return new_node
      
        
        
        
        
     #generates a new point towards a randomly sampled point from the nearest node
    def move_towards_point(self,nearest_node, random_point):
        #compute direction vector from nearest point to random point
      
    
        direction=np.array(random_point[:2])-np.array(nearest_node['point'][:2])
         
        #calculate the distance (norm) from the nearest node to the random point.
        distance=np.linalg.norm(direction)
        
        if distance==0:
            return nearest_node['point'][:2] #the random point is already in nearest point 
        
        #normalize the direction vector
        direction_normalized=direction/distance
        
        #calculate the new point
        #constrained by the maximum step size, delta_q
        new_point=nearest_node['point'][:2]+min(self.delta_q,distance)*direction_normalized
        
        #check if the new point is in collision
        if self.state_validity_checker.is_valid(new_point):
           return new_point
        else:
            return None
   

    
    
    

    def construct_path(self, q_start, q_goal):
        # Implement RRT algorithm.
        # Use the state_validity_checker object to see if a position is valid or not.
        
        # initialize the tree with the start point
        self.nodes = [{'point': np.array(q_start)[:2], 'parent': None}]
        q_goal=np.array(q_goal)[:2]
        for i in range(self.max_iterations):
             # decide whether to aim for the goal or sample randomly
            if np.random.rand() > self.p_goal:
                random_point = self.create_random_point()
            else:
                random_point = q_goal
             # find the nearest existing node in the tree to the sampled point 
            nearest_point = self.find_nearest_node(random_point)
            print('nearest point',nearest_point)
             
             
            # generate a new point moving towards the sampled point from the nearest point
            new_point = self.move_towards_point(nearest_point, random_point)
            
            
            # If the new point is valid or in collision
            if new_point is not None and self.state_validity_checker.is_valid(new_point):
                # add the new point to the tree
                new_node = self.add_node(new_point, nearest_point['point'])
            
                #check if we're close enough to the goal and if direct connection to goal is valid
                if np.linalg.norm(new_node['point'] - q_goal) <= self.delta_q and self.state_validity_checker.is_valid(q_goal):
                    #goal as a node
                    goal_node= self.add_node(q_goal, new_point)
                    # Retrace the path from goal to start
                    return self.retrace_path(goal_node)
            
        # if the loop ends without reaching the goal, return an empty list as failure
        return []
    
     
     #retraces the path from the goal node to the start node.
    def retrace_path(self, goal_node):
        #initialize the path
        current_path = [goal_node['point']] 
        current_parent = goal_node['parent']
        #continue retracing until there is no parent
        while current_parent is not None:
            # initialize parent_node
            parent_node = None
            for node in self.nodes:
                #if the current point node  matches the current_parent
                if np.array_equal(node['point'], current_parent):

                    parent_node = node
                    break
            #if parent node is None
            if parent_node is None:
                    break
                
                
            
            # add the found parent node to the current path.
            current_path.append(parent_node['point'])
        
            # update current_parent to the parent 
       
            current_parent = parent_node['parent']
    
    
        # starting from the start node and ending at the goal node.
        #reverse the path
        return current_path[::-1]
    
   
    def smooth_path(self, path):
        if not path or len(path) < 4:
            return path

        smoothed_path = [path[0]]
        i = 0
        
        while i < len(path) - 2:
            point = path[i]
            next_point = path[i + 2]
            
            if self.state_validity_checker.is_valid(point) and self.state_validity_checker.is_valid(next_point):
                smoothed_path.append(next_point)
                i += 2
            else:
                smoothed_path.append(path[i + 1])
                i += 1
                
        if not np.array_equal(smoothed_path[-1], path[-1]):
            smoothed_path.append(path[-1])
            
        return smoothed_path


    

            
        
                
                

        
    


# Planner: This function has to plan a path from start_p to goal_p. To check if a position is valid the 
# StateValidityChecker class has to be used. The planning dominion must be specified as well as the maximum planning time.
# The planner returns a path that is a list of poses ([x, y]).
def compute_path(start_p, goal_p, state_validity_checker,bounds,max_time=1.0,tolerance=0.1):

    # TODO: Plan a path from start_p to goal_p inside bounds using the RRT and the 
    # StateValidityChecker Objects previously defined.
    # TODO: if solved, return a list with the [x, y] points in the solution path.
    # example: [[x1, y1], [x2, y2], ...]
    # TODO: Ensure that the path brings the robot to the goal (with a small tolerance)!
    
    
    rrt = RRT(state_validity_checker=state_validity_checker, delta_q=4, p_goal=0.2, dominion=bounds)
    
    # record the start time 
    start_time = time.time()
    
    
    # initialize an empty path
    path = []
    
    # run the RRT algorithm until a path is found or the maximum time is reached
    while time.time() - start_time < max_time:
        # construct a path
        path = rrt.construct_path(start_p, goal_p)
        print('total_path',path)
        
        # if a path is found, break the loop
        if path:
            break
    
    # smooth path
    if path:
        path = rrt.smooth_path(path)
        print('smoothed_path',path)
        
        
        #check if the last point in the path is within the tolerance of the goal point
        last_point = path[-1]
        # if the last point is not within tolerance of the goal, adjust the path.
        if np.linalg.norm(np.array(last_point) - np.array(goal_p)) > tolerance:
            
            if state_validity_checker.is_valid(goal_p):
                path.append(goal_p)
            else:
                
                print("Direct extension to the goal is not valid. A more sophisticated adjustment is required.")
    
        
      
   
    return path
              
    
   


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

