# -*- coding: utf-8 -*-
"""
Created on Sat Nov 23 13:58:04 2019

@author: anup
"""

import math
import numpy
from matplotlib import pyplot as plt
import cv2
import Utils
import time
import random

class HaltonPlanner(object):
  
  # planningEnv: Should be a HaltonEnvironment
  def __init__(self, planningEnv):
    self.planningEnv = planningEnv

  # Generate a plan
  # Assumes that the source and target were inserted just prior to calling this
  # Returns the generated plan
  def plan(self, planType):
    self.sid = self.planningEnv.graph.number_of_nodes() - 2 # Get source id
    self.tid = self.planningEnv.graph.number_of_nodes() - 1 # Get target id

    self.closed = {} # The closed list
    self.parent = {self.sid:None} # A dictionary mapping children to their parents
    self.open = {self.sid: 0 + self.planningEnv.get_heuristic(self.sid, self.tid)} # The open list
    self.gValues = {self.sid:0} # A mapping from node to shortest found path length to that node 
    self.planIndices = []
    self.cost = 0
    # ------------------------------------------------------------
    # YOUR CODE HERE
    # 
    # Implement A*
    # Functions that you will probably use
    # - self.get_solution()
    # - self.planningEnv.get_successors()
    # - self.planningEnv.get_distance()
    # - self.planningEnv.get_heuristic()
    # Note that each node in the graph has both an associated id and configuration
    # You should be searching over ids, not configurations. get_successors() will return
    #   the ids of nodes that can be reached. Once you have a path plan
    #   of node ids, get_solution() will compute the actual path in SE(2) based off of
    #   the node ids that you have found.
    #-------------------------------------------------------------

    # Loop until you find the end
    if planType == "astar":
      print ("Using A*")
      return self.AStar()
    elif planType == "lazyastar":
      print ("Using Lazy A*")
      return self.Lazy_AStar()
    elif planType == "lazysp":
      print ("Using lazy SP")
      return self.Lazy_SP()
    else:
      print ("Unknown plan")
  
  def AStar(self):
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
            return self.get_solution(current_vid) # Return reversed path
        
        # Generate children
        # Let the children of the currentNode equal the adjacent nodes
        children_vids = self.planningEnv.get_successors(current_vid)

        for child_vid in children_vids:
            # print("Visiting {}".format(child_vid))
            
            # check to see if there is a collision
            configCurrent = self.planningEnv.get_config(current_vid)
            configChild = self.planningEnv.get_config(child_vid)
            if not(self.planningEnv.manager.get_edge_validity(configCurrent, configChild)) :
              continue

            # alreadyVisited = False
            # Child is on the closed list
            # for closed_child_vid in self.closed:
                #if child_vid == closed_child_vid:
                #    alreadyVisited = True
                #    break
            #if alreadyVisited:
            #    continue

            if child_vid in self.closed.keys():
                continue
            
            # Create the f, g, and h values
            latest_child_vid_g_distance = self.gValues[current_vid] + self.planningEnv.get_distance(current_vid, child_vid)
            if (not(child_vid in self.gValues.keys()) or
                    latest_child_vid_g_distance < self.gValues[child_vid]):
                self.gValues[child_vid] = latest_child_vid_g_distance
                self.parent[child_vid] = current_vid
                
            child_vid_h_distance = self.planningEnv.get_heuristic(child_vid, self.tid)
            child_vid_f_distance = self.gValues[child_vid] + child_vid_h_distance
            

            # Child is already in the open list
            # for open_vid in self.open:
            #    if child_vid == open_vid and self.gValues[child_vid] > self.gValues[open_vid]:
            #        continue
            
            if child_vid in self.open.keys():
                if self.open[child_vid] > child_vid_f_distance:
                    self.open[child_vid] = child_vid_f_distance
            else:
                # Add the child to the open list
                self.open[child_vid] = child_vid_f_distance    

    return []

  def Lazy_AStar(self):
    while bool(self.open):
        
        # Get the current node
        # let the currentNode equal the node with the least f value
        current_vid = next(iter(self.open))
        current_vid_fvalue = self.open[current_vid]
        for vid, f_value in self.open.items(): 
            if f_value < current_vid_fvalue:
                current_vid = vid
                current_vid_fvalue = f_value
              
        # check to see if there is a collision
        hasParent  = current_vid in self.parent.keys()

        if hasParent and self.parent[current_vid] != None:
          parent_vid = self.parent[current_vid]
          configCurrent = self.planningEnv.get_config(current_vid)
          # print(parent_vid)
          configParent = self.planningEnv.get_config(parent_vid)
          if not(self.planningEnv.manager.get_edge_validity(configParent, configCurrent)) :
            self.open[current_vid] = float('inf')
            self.gValues[current_vid] = float('inf')
            continue
        else:
          pass
        
        # Pop current off open list, add to closed list
        self.open.pop(current_vid)
        self.closed[current_vid] = current_vid_fvalue        
        
        # Found the goal
        if current_vid == self.tid:
            return self.get_solution(current_vid) # Return reversed path
        
        # Generate children
        # Let the children of the currentNode equal the adjacent nodes
        children_vids = self.planningEnv.get_successors(current_vid)

        for child_vid in children_vids:
            
            # alreadyVisited = False
            # Child is on the closed list
            # for closed_child_vid in self.closed:
                #if child_vid == closed_child_vid:
                #    alreadyVisited = True
                #    break
            #if alreadyVisited:
            #    continue

            if child_vid in self.closed.keys():
                continue
            
            # Create the f, g, and h values
            latest_child_vid_g_distance = self.gValues[current_vid] + self.planningEnv.get_distance(current_vid, child_vid)
            if (not(child_vid in self.gValues.keys()) or
                    latest_child_vid_g_distance < self.gValues[child_vid]):
                self.gValues[child_vid] = latest_child_vid_g_distance
                self.parent[child_vid] = current_vid
                
            child_vid_h_distance = self.planningEnv.get_heuristic(child_vid, self.tid)
            child_vid_f_distance = self.gValues[child_vid] + child_vid_h_distance
            

            # Child is already in the open list
            # for open_vid in self.open:
            #    if child_vid == open_vid and self.gValues[child_vid] > self.gValues[open_vid]:
            #        continue
            
            if child_vid in self.open.keys():
                if self.open[child_vid] > child_vid_f_distance:
                    self.open[child_vid] = child_vid_f_distance
            else:
                # Add the child to the open list
                self.open[child_vid] = child_vid_f_distance    

    return []

  def Lazy_SP(self):
    self.verbotenset = set()
    possible_path = self.Lazy_SPInner()
    # print(possible_path)
    possible_path_collision = self.path_collisions(possible_path)
    # while possible_path has collisions
    while possible_path_collision:
      print ("Path has collisions still")
      self.closed = {} # The closed list
      self.parent = {self.sid:None} # A dictionary mapping children to their parents
      # The open list
      self.open = {self.sid: 0 + self.planningEnv.get_heuristic(self.sid, self.tid)} 
      self.gValues = {self.sid:0} # A mapping from node to shortest found path length to that node 
      self.planIndices = []
      self.cost = 0

      possible_path = self.Lazy_SPInner()
      possible_path_collision = self.path_collisions(possible_path)

      pass
    return self.get_solution(self.tid)
    pass

  def path_collisions (self, planNodes):
    print ("Nodes {}".format(len(planNodes)))
    if planNodes == None:
      return False
    if len(planNodes) <= 1:
      return False
    for i in range(len(planNodes) - 2):
      startConfig = self.planningEnv.get_config(planNodes[i])
      endConfig = self.planningEnv.get_config(planNodes[i+1])
      validEdge = self.planningEnv.manager.get_edge_validity(startConfig, endConfig)
      if not(validEdge):
        self.verbotenset.add(str(planNodes[i]) + "," +  str(planNodes[i+1]))
        self.verbotenset.add(str(planNodes[i+1]) + "," +  str(planNodes[i]))
        print("Found collision")
        return True
      pass
    return False
    pass

  def get_solution_light(self, vid):
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

  def Lazy_SPInner(self):
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
            possible_path = self.get_solution_light(current_vid) # Return reversed path
            print(possible_path)
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
    print("Post process")
    t1 = time.time()
    elapsed = 0
    max = len(plan) - 1
    if max <= 0:
      return

    while elapsed < timeout: # Keep going until out of time
      # ---------------------------------------------------------
      # YOUR CODE HERE
      
      # Pseudocode
      
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
      viable = self.planningEnv.manager.get_edge_validity(src, target)

      if viable:
        px, py, clen = self.planningEnv.manager.discretize_edge(src, target)
        ptheta = numpy.zeros(len(px)).tolist()
        # replacePlan = [list(a) for a in zip(px, py, ptheta)]
        angle = math.atan2(target[1]-src[1], target[0] - src[0])
        replacePlan = numpy.zeros((len(px), 3))
        replacePlan[:,0] = px
        replacePlan[:,1] = py
        replacePlan[:,2] = angle

        # repNp = np.Array(replacePlan)
        newPlan = plan[0:i]
        newPlan = numpy.append(newPlan, replacePlan, axis=0)
        #for i in range(len(replacePlan)):
        #    newPlan.append(replacePlan[i])
        # print("Repl plan: {}".format(replacePlan[0:2]))
        if (j == len(plan)):
          # newPlan = plan[0:i] + replacePlan
          pass
        else:
          # newPlan = plan[0:i] + replacePlan + plan[j+1:]
          newPlan = numpy.append(newPlan, plan[j+1:], axis=0)
        pass

      elapsed = time.time() - t1
    return newPlan

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
    # print("Flatplan: {} ... {} len {} cost {}".format(flatPlan[0:5], flatPlan[-5:-1], len(flatPlan), self.cost))
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
