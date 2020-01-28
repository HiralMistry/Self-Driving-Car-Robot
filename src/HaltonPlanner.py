import math
import numpy
from matplotlib import pyplot as plt
import cv2
import Utils
import time
import random
import timeit
from nav_msgs.srv import GetMap

# Set perferred planning method to 'True'
A_STAR = False
A_STAR_LAZY = True
LAZY_SP = False

class HaltonPlanner(object):
  
  # planningEnv: Should be a HaltonEnvironment
  def __init__(self, planningEnv):
    self.planningEnv = planningEnv

  # Generate a plan
  # Assumes that the source and target were inserted just prior to calling this
  # Returns the generated plan
  def plan(self):
    self.start = timeit.default_timer()
    self.sid = self.planningEnv.graph.number_of_nodes() - 2 # Get source id
    self.tid = self.planningEnv.graph.number_of_nodes() - 1 # Get target id

    self.closed = {} # The closed list
    self.parent = {self.sid:None} # A dictionary mapping children to their parents
    self.open = {self.sid: 0 + self.planningEnv.get_heuristic(self.sid, self.tid)} # The open list
    self.gValues = {self.sid:0} # A mapping from node to shortest found path length to that node 
    self.planIndices = []
    self.cost = 0

    # Plan with A*
    if A_STAR:
      while(len(self.open)!=0):
        current_node = min(self.open.keys(), key=(lambda k: self.open[k]))
        self.closed[current_node]=self.open[current_node]
        self.open.pop(current_node)
        if(current_node==self.tid):
          stop = timeit.default_timer()
          return self.get_solution(self.tid)
        successors = self.planningEnv.get_successors(current_node)
        for i in range(len(successors)):
          if(successors[i] in self.closed):
            continue
          if not(self.planningEnv.manager.get_edge_validity(self.planningEnv.get_config(current_node), self.planningEnv.get_config(successors[i]))):
            continue
          heuristic_cost = self.planningEnv.get_heuristic(successors[i], self.tid)
          g_val = self.gValues[current_node] + self.planningEnv.get_distance(current_node,successors[i])
          f_val = g_val +heuristic_cost
          if(successors[i] in self.open):
            if(self.gValues[successors[i]] > g_val):
              self.open[successors[i]] = f_val
              self.gValues[successors[i]] = g_val
              self.parent[successors[i]]= current_node
          else:
            self.open[successors[i]] = f_val
            self.gValues[successors[i]] = g_val
            self.parent[successors[i]]= current_node
    
    # Plan with Lazy A*
    if A_STAR_LAZY:
      while(len(self.open)!=0):
        current_node = min(self.open.keys(), key=(lambda k: self.open[k]))
        self.closed[current_node]=self.open[current_node]
        self.open.pop(current_node)
        if(current_node==self.tid):
          stop = timeit.default_timer()
          return self.get_solution(self.tid)
        successors = self.planningEnv.get_successors(current_node)
        for i in range(len(successors)):
          if(successors[i] in self.closed):
            continue
          heuristic_cost = self.planningEnv.get_heuristic(successors[i], self.tid)
          g_val = self.gValues[current_node] + self.planningEnv.get_distance(current_node,successors[i])
          f_val = g_val +heuristic_cost
          if not(self.planningEnv.manager.get_edge_validity(self.planningEnv.get_config(current_node), self.planningEnv.get_config(successors[i]))):
            continue
          if(successors[i] in self.open):
            if(self.gValues[successors[i]] > g_val):
              self.open[successors[i]] = f_val
              self.gValues[successors[i]] = g_val
              self.parent[successors[i]]= current_node
          else:
            self.open[successors[i]] = f_val
            self.gValues[successors[i]] = g_val
            self.parent[successors[i]]= current_node

    # Plan with Lazy SP
    if LAZY_SP:
				self.verbotenset = set()
				try_path = self.Lazy_SP_Helper()
				try_path_collision = self.path_collisions(try_path)
				while try_path_collision:
				  print ("Path has collisions still")
				  self.closed = {} # The closed list
				  self.parent = {self.sid:None} # A dictionary mapping children to their parents
				  # The open list
				  self.open = {self.sid: 0 + self.planningEnv.get_heuristic(self.sid, self.tid)} 
				  self.gValues = {self.sid:0} # A mapping from node to shortest found path length to that node 
				  self.planIndices = []
				  self.cost = 0

				  try_path = self.Lazy_SP_Helper()
				  try_path_collision = self.path_collisions(try_path)

				  pass
				return self.get_solution(self.tid)
				pass
  def path_collisions (self, planNodes):
    '''
    Takes the nodes of the pland and makes sure they are not in collision
    Returns false if there is no collision and true if there is a collision
    '''
    print ("Nodes {}".format(len(planNodes)))
    if planNodes == None:
      return False
    if len(planNodes) <= 1:
      return False
    for i in range(len(planNodes) - 2):
      Config1 = self.planningEnv.get_config(planNodes[i])
      Config2 = self.planningEnv.get_config(planNodes[i+1])
      validEdge = self.planningEnv.manager.get_edge_validity(Config1, Config2)
      if not(validEdge):
        self.verbotenset.add(str(planNodes[i]) + "," +  str(planNodes[i+1]))
        self.verbotenset.add(str(planNodes[i+1]) + "," +  str(planNodes[i]))
        print("Found collision")
        return True
      pass
    return False
    pass

  def get_solution_2(self, vid):
    # Get all the node ids
    planID = []
    while vid is not None:
      planID.append(vid)
      vid = self.parent[vid]
    print("Len: {}".format(len(planID)) )
    planID.reverse()
    plan = planID
    # plan = np.array(planID).tolist().reverse()
    print("Len: {}".format(len(plan)) )
    return plan
    pass

  def Lazy_SP_Helper(self):
    while bool(self.open):
        
        # Get the current node
        # let the currentNode equal the node with the least f value
        current_vid = next(iter(self.open))
        current_vid_fvalue = self.open[current_vid]
        for vid, f_value in self.open.items(): 
            if f_value < current_vid_fvalue:
                current_vid = vid
                current_vid_fvalue = f_value
                      
        # Pop current off open list, add to closed list
        self.open.pop(current_vid)
        self.closed[current_vid] = current_vid_fvalue        
        
        # Found the goal
        if current_vid == self.tid:
            possible_path = self.get_solution_2(current_vid) # Return reversed path
            stop = timeit.default_timer()
            return possible_path
        
        # Generate children
        # Let the children of the currentNode equal the adjacent nodes
        children_vids = self.planningEnv.get_successors(current_vid)

        for child_vid in children_vids:

            if child_vid in self.closed.keys():
              continue
            
            keyStartEnd = str(current_vid) + "," + str(child_vid)
            if (keyStartEnd in self.verbotenset):
              continue
            
            # Create the f, g, and h values
            latest_child_vid_g_distance = self.gValues[current_vid] + self.planningEnv.get_distance(current_vid, child_vid)
            if (not(child_vid in self.gValues.keys()) or
                    latest_child_vid_g_distance < self.gValues[child_vid]):
                self.gValues[child_vid] = latest_child_vid_g_distance
                self.parent[child_vid] = current_vid
                
            child_vid_h_distance = self.planningEnv.get_heuristic(child_vid, self.tid)
            child_vid_f_distance = self.gValues[child_vid] + child_vid_h_distance
            

            if child_vid in self.open.keys():
                if self.open[child_vid] > child_vid_f_distance:
                    self.open[child_vid] = child_vid_f_distance
            else:
                # Add the child to the open list
                self.open[child_vid] = child_vid_f_distance    

    return []
    pass


  # Try to improve the current plan by repeatedly checking if there is a shorter path between random pairs of points in the path

  def post_process(self, plan, timeout):
    t1 = time.time()
    elapsed = 0
    max = len(plan) - 1
    if max <= 0:
      return

    while elapsed < timeout: # Keep going until out of time
  
      # Pick random id i
      # Pick random id j
      # Redraw if i == j
      # Switch i and j if i > j
      i = random.randint(0,max)
      j = random.randint(0,max)
      
      if i == j:
        continue
      
      if i > j:
        temp = j
        j = i
        i = temp

      # if we can find path between i and j (Hint: look inside ObstacleManager.py for a suitable function)
        # Get the path
        # Reformat the plan such that the new path is inserted and the old section of the path is removed between i and j
        # Be sure to CAREFULLY inspect the data formats of both the original plan and the plan returned
        # to ensure that you edit the path correctly

      src = plan[i]
      target = plan[j]
      valid = self.planningEnv.manager.get_edge_validity(src, target)

      if valid:
        px, py, clen = self.planningEnv.manager.discretize_edge(src, target)
        ptheta = numpy.zeros(len(px)).tolist()
        angle = math.atan2(target[1]-src[1], target[0] - src[0])
        new_plan = numpy.zeros((len(px), 3))
        new_plan[:,0] = px
        new_plan[:,1] = py
        new_plan[:,2] = angle

        copy_array = plan[0:i]
        copy_array = numpy.append(copy_array, new_plan, axis=0)
        if (j == len(plan)):
          pass
        else:
          copy_array = numpy.append(copy_array, plan[j+1:], axis=0)
        pass

      elapsed = time.time() - t1
    return copy_array

  # Backtrack across parents in order to recover path
  # vid: The id of the last node in the graph
  def get_solution(self, vid):

    # Get all the node ids
    planID = []
    while vid is not None:
      planID.append(vid)
      vid = self.parent[vid]

    plan = []
    planID.reverse()
    for i in range(len(planID) - 1):
      startConfig = self.planningEnv.get_config(planID[i])
      goalConfig = self.planningEnv.get_config(planID[i + 1])
      px, py, clen = self.planningEnv.manager.discretize_edge(startConfig, goalConfig)
      plan.append([list(a) for a in zip(px, py)])
      self.planIndices.append(len(plan))
      self.cost += clen

    flatPlan = [item for sublist in plan for item in sublist]
    return flatPlan

  # Visualize the plan
  def simulate(self, plan):
    # Get the map
    envMap = 255*(self.planningEnv.manager.mapImageBW+1) # Hacky way to get correct coloring
    envMap = cv2.cvtColor(envMap, cv2.COLOR_GRAY2RGB)
    
    for i in range(numpy.shape(plan)[0]-1): # Draw lines between each configuration in the plan
      startPixel = Utils.world_to_map(plan[i], self.planningEnv.manager.map_info)
      goalPixel = Utils.world_to_map(plan[i+1], self.planningEnv.manager.map_info)
      cv2.line(envMap,(startPixel[0],startPixel[1]),(goalPixel[0],goalPixel[1]),(255,0,0),5)

    # Generate window
    cv2.namedWindow('Simulation', cv2.WINDOW_NORMAL)
    cv2.imshow('Simulation', envMap)

    # Terminate and exit elegantly
    cv2.waitKey(20000)
    cv2.destroyAllWindows()

if __name__ == '__main__':
  rospy.init_node('obstacle_manager', anonymous=True)
  car_width = rospy.get_param("/car_kinematics/car_width", 0.33)
  car_length = rospy.get_param("/car_kinematics/car_length", 0.33)
  collision_delta = rospy.get_param("~collision_delta", 0.05)
  map_service_name = rospy.get_param("~static_map", "static_map")
  rospy.wait_for_service(map_service_name)
  map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map

