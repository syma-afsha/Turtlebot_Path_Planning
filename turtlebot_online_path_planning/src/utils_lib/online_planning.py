import numpy as np
import time
import random
import math
from math import sqrt
def wrap_angle(angle):
    #corrects an angle to be within the range of [-pi, pi]
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

class StateValidityChecker:
    def __init__(self, distance=0.2, is_unknown_valid=True):
        self.map = None
        self.resolution = None
        self.origin = None
        self.there_is_map = False
        self.distance = distance
        self.is_unknown_valid = is_unknown_valid

    def set(self, data, resolution, origin):
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
        self.height = data.shape[0]
        self.width = data.shape[1]

    def is_valid(self, pose):
        # Check if the occupancy map is set before proceeding.
        if not self.there_is_map:
            raise ValueError("Occupancy map not set.")

        # Convert the given world pose to map coordinates.
        map_x = round((pose[0] - self.origin[0]) / self.resolution)
        map_y = round((pose[1] - self.origin[1]) / self.resolution)

        # Calculate the number of grid cells to check around the pose based on the specified distance.
        search_radius_in_cells = int(self.distance / self.resolution)

        # Calculate the lower bounds for the search area in map coordinates.
        search_area_lower_x = map_x - search_radius_in_cells
        search_area_lower_y = map_y - search_radius_in_cells

        # Iterate through the grid cells in the vicinity of the pose to check for occupancy.
        for offset_x in range(search_area_lower_x, search_area_lower_x + 2 * search_radius_in_cells + 1):
            for offset_y in range(search_area_lower_y, search_area_lower_y + 2 * search_radius_in_cells + 1):
                # Check if the current cell is within the bounds of the map.
                if 0 <= offset_x < self.height and 0 <= offset_y < self.width:
                    cell_value = self.map[offset_x, offset_y]  # Retrieve the occupancy value of the current cell.
                    # Determine if the current cell is not free (occupied or unknown).
                    if cell_value != 0:  # Non-zero indicates either occupied or unknown space.
                        if cell_value == -1 and not self.is_unknown_valid:
                            # The cell is unknown and unknown spaces are not considered valid.
                            return False
                        elif cell_value != -1:
                            # The cell is occupied by an obstacle.
                            return False
                else:
                    # The current cell is outside the map bounds and unknown spaces are not considered valid.
                    if not self.is_unknown_valid:
                        return False

        # The pose and its vicinity are considered valid based on the occupancy map and the settings.
        return True
    
    

    # Given a path, returns true if the path is not in collision and false otherwise
    def check_path(self, path):
        """
        Check if a path consisting of waypoints is valid by discretizing each segment and
        verifying each discretized point is within a valid area.

        Args:
            path (list of tuples): List of waypoints defining the path.

        Returns:
            bool: True if the entire path is valid, False if any segment or point is invalid.
        """
        step_size = 0.5 * self.distance  # Discretization step size.

        for index in range(len(path) - 1):
            start_pose = path[index]
            end_pose = path[index + 1]

            # Calculate Euclidean distance between the current and next waypoint.
            distance = math.sqrt((end_pose[0] - start_pose[0])**2 + (end_pose[1] - start_pose[1])**2)
            
            if distance == 0:  # Avoid division by zero if the points are identical.
                direction_vector = np.array([0, 0])
            else:
                # Compute normalized direction vector between the two waypoints.
                direction_vector = np.array([end_pose[0] - start_pose[0], end_pose[1] - start_pose[1]]) / distance

            # Start discretizing the segment.
            current_pose = np.array(start_pose)

            # Check each discretized point along the segment.
            while math.sqrt((end_pose[0] - current_pose[0])**2 + (end_pose[1] - current_pose[1])**2) > step_size:
                current_pose += direction_vector * step_size
                # Check if the current discretized point is valid.
                if not self.is_valid(current_pose):
                    return False  # Return False if any point along the segment is invalid.

            # Finally, check the end waypoint of the current segment.
            if not self.is_valid(end_pose):
                return False  # Return False if the end waypoint is invalid.

        # If all waypoints and discretized points are valid, return True.
        return True
    # Transform position with respect to the map origin to cell coordinates
    
    
    ###################   RRT  ################################
          
class RRT:
    def  __init__(self,start,goal,state_validity_checker, max_iterations=10000, delta_q=4, p_goal=0.2, dominion=[-10, 10, -10, 10]):
        self.state_validity_checker=state_validity_checker
        self.max_iterations=max_iterations
        self.delta_q=delta_q
        self.p_goal=p_goal
        self.dominion=dominion
        self.start=start
        self.goal_p=goal
        self.nodes=[{'point':np.array(self.start)[:2], 'parent': None, 'cost':0}]
        
        
    
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
                distance = np.linalg.norm(np.array(node['point'])[:2]- np.array(random_point))
                #if the calculated distance is smaller than the current minimum distance then update the nearest node and minimum distance
                if distance < min_distance:
                    nearest_node=node
                    min_distance=distance
               
        return nearest_node
    def determine_new_point(self,nearest_point,random_point):
        # print('point',type(nearest_point))
        # print('ran',type(random_point))
        direction= np.array(random_point) - np.array(nearest_point)
        distance=np.linalg.norm(direction)
        direction_vector = direction / distance
        if self.delta_q>distance:
            return random_point
        new_point=np.array(nearest_point) + direction_vector* self.delta_q
        
        # print('new point',new_point)
        return new_point
        
   
    def add_new_node(self, new_point, parent_node, new_node_cost):
    

        if parent_node and 'point' in parent_node:
            parent_point = parent_node['point']
        else:
            # If parent_node is None or improperly structured, assume no parent (should only happen for the root node)
            parent_point = None
            new_node_cost = 0  # Cost for root node should be 0

        # Create the new node dictionary with the point, parent, and cost
        new_node = {
            'point': new_point,  # New node coordinates
            'parent': parent_point,  # Parent node coordinates or None for the root node
            'cost': new_node_cost  # Total cost from the start node to this node
        }

        # Append the new node to the list of nodes
        self.nodes.append(new_node)

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

    def is_close(self, point_a, point_b, tolerance=0.01):
        # Check if two points are close to each other within a given tolerance.
        if point_a is None or point_b is None:
            return False
        close_two_points=np.linalg.norm(np.array(point_a) - np.array(point_b)) <= tolerance
        print('Checking if two points are close to each other',close_two_points)
        return close_two_points
    def smooth_path(self, path, start, goal):
        counter = 0
        max_iterations = 10000
        smoothed_path = [goal]

        while True:
            for point in reversed(path[:-1]):  # Exclude the goal itself since it's already in smoothed_path
                if self.state_validity_checker.check_path([point, smoothed_path[-1]]):
                    smoothed_path.append(point)
                    break  # Exit the loop once a direct path is found to any point.

             # Use the is_close method to check if the last point in smoothed_path is close enough to the start.
            if self.is_close(smoothed_path[-1], start):
                break

            counter += 1
            if counter >= max_iterations:
                print("Max iteration reached")
                smoothed_path = path  # Fallback to the original path if too many iterations.
                break

        smoothed_path.reverse()  # The path was built backwards; reverse it to start-to-goal order.
        return smoothed_path
   
    def find_near_nodes(self, new_node, max_dist):
        near_nodes = []  # Initialize an empty list to store nodes that are found to be near the new_node.

    # Iterate through each node currently in the tree to check its distance from the new_node.
        for node in self.nodes:
            # Calculate the Euclidean distance between the new_node and the current node.
            if np.linalg.norm(new_node - node['point'][:2]) <= max_dist:
                # If the distance is less than or equal to max_dist, the node is considered near and added to the list.
                near_nodes.append(node)

        return near_nodes  # Return the list of nodes that are near to the new_node.

    
        
    def rewire(self, new_node, near_nodes):
        for node in near_nodes:
        # Calculate the potential new cost to reach 'node' via 'new_node'.
            new_cost = new_node['cost'] + np.linalg.norm(new_node['point'][:2] - node['point'][:2])
            
        # Check if the new cost is lower and the segment between new_node and node is free of obstacles.
            if new_cost < node['cost'] and self.check_segment(new_node['point'][:2], node['point'][:2]):
                # Update 'node' to have 'new_node' as its parent.
                node['parent'] = new_node['point']
                node['cost'] = new_cost  # Update the cost to the new, lower cost.
                
                # Recursively update costs for all descendants of 'node', as their path costs may also change.
                self.update_descendant_costs(node)
    def update_descendant_costs(self, node):
    # Recursively updates the costs of all descendants of a node.
    #this method is  called after a node's parent has been changed to ensure the path costs
    # of its descendants reflect the new path to the start node.

        for child in self.find_children(node):
            child['cost'] = node['cost'] + np.linalg.norm(child['point'] - node['point'])
            self.update_descendant_costs(child)  # Recursively update costs for descendants
    def find_children(self, parent_node):
         # Return a list of nodes whose 'parent' matches the parent_node's 'point'.
        return [node for node in self.nodes if np.array_equal(node['parent'],parent_node)]


              
                
           
    def compute_path(self,qgoal):
        qgoal = np.array(qgoal)
        for _ in range(self.max_iterations):
            qrand=self.find_valid_random_point()
            qnear=self.find_nearest_node(qrand)
            qnew=self.determine_new_point(qnear['point'][:2],qrand)
            check_segment_new_near=self.check_segment(qnew,qnear['point'][:2])
           
            if check_segment_new_near:
                near_nodes=self.find_near_nodes(qnew,max_dist=4)
                 # Initialize minimum cost to reach qnew through a direct connection from qnear
                min_cost = qnear['cost'] + np.linalg.norm(qnew - qnear['point'])
                qnew_parent = qnear
                qnew_cost = min_cost

                # Check for a cheaper path to qnew through any of the nearby nodes
                for node in near_nodes:
                    if self.check_segment(qnew, node['point']):
                        potential_new_cost = node['cost'] + np.linalg.norm(qnew - node['point'])
                        if potential_new_cost < min_cost:
                            qnew_parent = node
                            qnew_cost = potential_new_cost
                            min_cost = potential_new_cost

                  # Add the new node to the tree with its parent and updated cost
                new_node = self.add_new_node(qnew, qnew_parent, qnew_cost)
             
                # print('new_node',new_node)
                self.rewire(new_node, near_nodes)
                if np.linalg.norm(new_node['point'] - qgoal) <= self.delta_q and self.check_segment(new_node['point'], qgoal):
                     goal_node_cost = new_node['cost'] + np.linalg.norm(qgoal - new_node['point'])
                     goal_node = self.add_new_node(qgoal,new_node,goal_node_cost)
                    #  print('goal_node',goal_node)
                    #  print(self.nodes)
                     total_path=self.retrace_path(goal_node)
                    #  print('t',total_path)
                     if self.state_validity_checker.check_path(total_path):
                        return total_path
                     start, goal = total_path[0], total_path[-1]
                    #  print('Total path',total_path)
                    #  print('start',start)
                    #  print('end',goal)
                     return self.smooth_path(total_path,start,goal)
        return []      
              
                
           


def compute_path(start_p, goal_p, state_validity_checker, bounds):
    # Create an instance of RRT with the given parameters
    rrt = RRT(start_p,goal_p, state_validity_checker=state_validity_checker, max_iterations=10000, delta_q=4, p_goal=0.2, dominion=bounds)

   
    
    # run the RRT algorithm until a path is found or the maximum time is reached
    
    path = rrt.compute_path(goal_p)
            
        
    return path  # Return None if no path is found within the maximum time
    
    


# Controller: Given the current position and the goal position, this function computes the desired 
# lineal velocity and angular velocity to be applied in order to reah the goal.
# def move_to_point(current, goal, Kv=0.5, Kw=0.5):

def move_to_point(current, goal, Kv=0.5, Kw=0.5, reverse_threshold=math.pi/2):
    # Calculate the Euclidean distance to the goal.
    distance = math.sqrt((goal[0] - current[0]) ** 2 + (goal[1] - current[1]) ** 2)
    # Desired orientation to reach the goal from the current position.
    desire_oreientation = math.atan2(goal[1] - current[1], goal[0] - current[0])
    # Angular correction needed, wrapped within [-pi, pi].
    angular_correction = wrap_angle(desire_oreientation - current[2])

    # Determine the most efficient way to reach the goal (forward or backward).
    # If the absolute value of the angular correction is less than the reverse threshold,
    # it means moving forward is efficient. Otherwise, consider reversing.
    if abs(angular_correction) <= reverse_threshold:
        # Move forward
        v = Kv * distance
        w = Kw * angular_correction
    else:
        # Reverse movement
        # Adjust orientation for reverse; the goal is essentially "behind" the robot.
        angular_correction = wrap_angle(desire_oreientation - current[2] + math.pi)
        v = -Kv * distance  # Negative velocity for reverse
        w = Kw * angular_correction

    # Ensure robot is oriented correctly before moving forward or backward.
    if abs(angular_correction) > 0.05:
        v = 0  # Prioritize orientation correction

    return v, w