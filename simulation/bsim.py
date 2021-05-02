#!/usr/bin/env python

import time
import random
import sys
import pickle
import string
import numpy as np
import logging
import time
import random
import math
import scipy
import matplotlib.pyplot as plt
from numpy.linalg import norm
from scipy.spatial.distance import cdist, pdist, euclidean

from matplotlib import animation, rc
import simulation.environment as environment



###########################################################################################



class swarm(object):

    def __init__(self):

        self.agents = []
        self.size = 0
        self.speed = 0.7*np.ones(self.size)
        self.behaviour = 'none'

        self.centroids = []
        self.centermass = [0,0]
        self.median = [0,0]
        self.upper = [0,0]
        self.lower  = [0,0]
        self.spread = 0
        self.radius = 0.1
        
        self.max_load = 6
        self.load_threshold = 0.6

        self.field = []
        self.grid = []

        self.param = 3
        self.map = 'none'
        self.beacon_att = np.array([[]])
        self.beacon_rep = np.array([[]])

        self.origin = np.array([-80,0])
        self.start = np.array([])

        self.holding = []
        self.boxnum = []

        self.headings = []
        self.shadows = []
        
        self.lost = []
        
        self.verbose = False
        
        


    def gen_agents(self):

        dim = 0.001
        self.speed = 1*np.ones(self.size)
        self.agents = np.zeros((self.size,2))
        self.holding = np.zeros(self.size)
        self.lost = np.zeros(self.size)
        self.managing = np.zeros(self.size)
        self.task = np.zeros(self.size)
        self.force = 0.5*np.ones(self.size)
        self.foraging = 0.5*np.ones(self.size)
        self.boxnum = np.zeros((self.size, int(self.max_load * 3)))- 1
        self.countdown = np.zeros(self.size)
        self.headings = 0.0314*np.random.randint(-100,100 ,self.size)
        for n in range(self.size):
            self.agents[n] = np.array([dim*n - (dim*(self.size-1)/2) + self.origin[0], 0 + self.origin[1]])

        self.shadows = np.zeros((4,self.size,2))

    
    
    def get_state(self):

        totx = 0; toty = 0; totmag = 0
        # Calculate connectivity matrix between agents
        mag = cdist(self.agents, self.agents)
        totmag = np.sum(mag)
        #totpos = np.sum(self.agents, axis=0)

        # calculate density and center of mass of the swarm
        self.spread = totmag/((self.size -1)*self.size)
        # self.centermass[0] = (totpos[0])/(self.size)
        # self.centermass[1] = (totpos[1])/(self.size)
        self.median = np.median(self.agents, axis = 0)
        # self.upper = np.quantile(self.agents, 0.75, axis = 0)
        # self.lower = np.quantile(self.agents, 0.25, axis = 0)

        # Shift shadows
        
        # for n in range(len(self.shadows)):
        #   print('\nshadows length %d \n' % len(self.shadows)) 
        #   self.shadows[len(self.shadows)-n] = self.shadows[len(self.shadows)-n-1]
        self.shadows[3] = self.shadows[2]
        self.shadows[2] = self.shadows[1]
        self.shadows[1] = self.shadows[0]
        self.shadows[0] = self.agents
        
        
                
                
               
        
    def update_tasks(self, boxes, demand):
        for n in range(0, len(self.agents)):
            # if agent is carrying more than the allowed weight, turn red and stop
            if self.holding[n] == 0 and self.managing[n] == 0 and self.countdown[n] == 0:
            # if near a box that has been collected
                                # which box is closest?
                # is box close enough to forage?
                #if mag[0][closest] < 1 and boxes.collected[closest] == 1 and boxes.managed[closest] == 0 :# and self.prob_pickup > random.random():
                if (boxes.collection_origin - boxes.collection_size) <= self.agents[n][0] <= (boxes.collection_origin + boxes.collection_size) and (boxes.collection_origin - boxes.collection_size) <= self.agents[n][1] <= (boxes.collection_origin + boxes.collection_size):
                    # decide if foraging given demand
                    
                    p_foraging = 1/(1 + np.exp(6*(demand - self.foraging[n])))
                    
                    if p_foraging > random.random():
                        self.task[n] = 0
                        self.foraging[n] += 0.05
                        self.countdown[n] = 10
                        #print('should i forage? yes. threshold is ', self.foraging[n], 'demand is ', demand)
                    else:
                        #print('should i forage? no. threshold is ', self.foraging[n], 'demand is ', demand)
                        self.task[n] = 1
                        self.foraging[n] -= 0.05
                        self.countdown[n] = 15
                    
                    
            
            # keep foraging threshold between 0 and 1
            
            if self.foraging[n] > 1:
                self.foraging[n] = 0.95
            if self.foraging[n] < 0:
                self.foraging[n] = 0.05
                
                    
            if self.countdown[n] > 0:
                self.countdown[n] -= 1
            

    

# Functions and definitions for set of box objects.

class base(object):

    def __init__(self):
        self.pos = []
        self.speed = 0
        self.radius = 10
        self.move_radius = 50
        self.dropoffs = 0
        
    def set_state(self, initial_location = [0,0], speed = 0):
        self.pos = initial_location
        self.speed = speed
        
    def update_pos(self, movement):
        self.pos = [self.pos[0] + movement, self.pos[1]]
    

class boxes(object):

    def __init__(self):
        self.boxes = []
        self.radius = 1
        self.picked = []
        self.collected = []
        self.broadcast = []
        self.choice = 0
        self.sequence = False
        self.coverage = 0
        self.collection_size = 10
        self.tot_collected = 0
        self.collection_origin = 40
        self.prob_pickup = 1
        self.prob_dropoff = 0.2
        self.map_nest = map()
        #self.map_nest.empty(self.collection_size+2, [self.collection_origin, self.collection_origin])
        self.map_nest.gen()
        self.from_base = []

    def set_state(self, state, dim_w, dim_h, num):
 
        if state == 'random':
            heights = np.random.randint(-dim_w, dim_w, (num,1))
            widths = np.random.randint(-dim_h, dim_h, (num,1))
            self.boxes = np.hstack((heights, widths))
            #self.boxes = np.random.randint(-47,47, (50,2))
            self.picked = np.zeros(len(self.boxes))
            self.collected = np.zeros(len(self.boxes))
            self.managing = np.zeros(len(self.boxes))
            self.managed = np.zeros(len(self.boxes))
            self.broadcast = np.zeros(len(self.boxes))
            # Randomly set the first box to be collected
            self.broadcast = list(range(0, len(self.boxes)))
            self.choice = random.randint(0, len(self.boxes))
            self.from_base = np.zeros(len(self.boxes))



    # Check the state of the boxes 
    def get_state(self):
        score = np.sum(self.collected)

        done = np.sum(self.managed)
        
        demand = np.sum(self.collected) - np.sum(self.managed)
        demand = demand/len(self.boxes)
        
        return score, done, demand    
    
    def update_boxes(self, swarm, base, t):
      
        score = 0
        # adjacency matrix of agents and boxes
        mag = cdist(swarm.agents, self.boxes)
        from_base = cdist(swarm.agents, [base.pos])
        from_base_box = cdist(self.boxes, [base.pos])
        # Check which distances are less than detection range
        a = mag < self.radius
        # Sum over agent axis 
        detected = np.sum(a, axis = 1)
        # convert to boolean, targets with 0 detections set to false.
        detected = detected > 0


        for n in range(0, len(swarm.agents)):
            picked = False
            # if the agent is in "foraging mode"
            if swarm.task[n] == 0:
                
                # which box is closest?
                #closest = np.where(np.amin(mag[n]) == mag[n])
                mag = cdist([swarm.agents[n]], self.boxes)
                while np.any(mag < self.radius):
                    closest = np.argmin(mag)                    
                    # is box close enough pick up?
                    if mag[0][closest] < self.radius and self.picked[closest] != 1 and self.collected[closest] != 1 and self.prob_pickup > random.random():
                        # box has been picked
                        picked = True
                        if swarm.verbose == True:
                            print('agent ', n, 'collected ', swarm.holding[n])
                        swarm.boxnum[n, int(swarm.holding[n])] = closest 
                        
                        self.picked[closest] = 1
  
                        self.boxes[closest, :] = swarm.agents[n]

                        swarm.holding[n] += 1
                        
                    mag[0][closest] = 50

                # If agent is holding a box update its position
                if swarm.holding[n] > 0 and picked == False:
                    #print('agent ', n, 'is holding a box')
                    #print(swarm.boxnum[n,:])
                    #print([x for x in swarm.boxnum[n,:] if x != -1])
                    self.boxes[[int(x) for x in swarm.boxnum[n,:] if x != -1]] = swarm.agents[n]
                    # Is agent in within dropoff zone
                    if from_base[n] < base.radius and self.prob_dropoff > random.random():
                        the_boxes = [int(x) for x in swarm.boxnum[n,:] if x != -1]
                        if swarm.verbose == True:
                            print('boxes dropped', [int(x) for x in swarm.boxnum[n,:] if x != -1], self.boxes[[int(x) for x in swarm.boxnum[n,:] if x != -1]], 'by agent', n)
                        for b in the_boxes:
                            #if from_base_box[b] < base.radius:
                            #print('in base!', np.shape(self.collected), x for x in swarm.boxnum[n,:] if x != -1])
                            #self.collected[[int(x) for x in swarm.boxnum[n,:] if x != -1]] = 1
                            self.collected[b] = 1
                            self.from_base[b] = base.pos[0] - self.boxes[b,0]
                            #else:
                                #print("close to agent but not box! agent", n, "box", b)
                        
                            #dist = cdist(self.boxes, [base.pos])
                            #self.from_base[[int(x) for x in swarm.boxnum[n,:] if x != -1]] = dist[[int(x) for x in swarm.boxnum[n,:] if x != -1],0] 
                        
                        swarm.holding[n] = 0
                        base.dropoffs += 1


                 
            
        self.tot_collected += np.sum(self.collected)

    #def check_boxes(self, base):
    
        #for boxn in range(len(self.boxes)):
        #    if self.collected[boxn] == 1 and 
        
    def update_pos(self, movement):
        #print(self.boxes[self.collected == 1, :] )
        #print(self.boxes[self.collected == 1,0], self.boxes[self.collected == 1,1])
        move = movement + np.zeros((len(self.boxes)))
        #move = np.hstack((move, np.zeros((len(self.boxes)))))
        #print(np.shape(self.boxes))
        #print(np.shape(move*self.collected))
        #print(np.shape(np.transpose(np.vstack((move*self.collected, np.zeros((len(self.boxes))))))))
        #print(np.transpose(np.vstack((move*self.collected, np.zeros((len(self.boxes)))))))
        self.boxes = self.boxes + np.transpose(np.vstack((move*self.collected, np.zeros((len(self.boxes))))))
        #self.boxes[self.collected == 1, 0] = self.from_base[self.collected == 1] + base.pos[0]



    def reset(self):
        pass




# Environment objects which are used within the map class. 

class make_wall(object):

    def __init__(self):

        self.start = np.array([0, 0])
        self.end = np.array([0, 0])
        self.width = 1
        self.hitbox = []


class make_box(object):

    def __init__(self, h, w, origin):

        self.height = h
        self.width = w
        self.walls = []

        self.walls.append(make_wall())
        #print(origin, h, w)
        #print(origin[0]-(0.5*w), origin[1]+(0.5*h), origin[1]+(0.5*w), origin[1]+(0.5*h))
        self.walls[0].start = [int(origin[0]-(0.5*w)), int(origin[1]+(0.5*h))]; self.walls[0].end = [int(origin[0]+(0.5*w)), int(origin[1]+(0.5*h))]
        self.walls.append(make_wall())
        self.walls[1].start = [origin[0]-(0.5*w), origin[1]-(0.5*h)]; self.walls[1].end = [origin[0]+(0.5*w), origin[1]-(0.5*h)]
        self.walls.append(make_wall())
        self.walls[2].start = [origin[0]-(0.5*w), origin[1]+(0.5*h)]; self.walls[2].end = [origin[0]-(0.5*w), origin[1]-(0.5*h)]
        self.walls.append(make_wall())
        self.walls[3].start = [origin[0]+(0.5*w), origin[1]+(0.5*h)]; self.walls[3].end = [origin[0]+(0.5*w), origin[1]-(0.5*h)]
        



# This class contains definitions for different envionments that can be spawned

class map(object):

    def __init__(self):

        self.obsticles = []
        self.force = 0
        self.walls = np.array([])
        self.wallh = np.array([])
        self.wallv = np.array([])
        self.planeh = np.array([])
        self.planev = np.array([])

    def copy(self):
        newmap = map()
        newmap.walls = self.walls[:]
        newmap.wallh = self.wallh[:]
        newmap.wallv = self.wallv[:]
        newmap.planeh = self.planeh[:]
        newmap.planev = self.planev[:]
        newmap.limh = self.limh[:]
        newmap.limv = self.limv[:]
        newmap.gen()
        return newmap

    def gen(self):

        # Perform pre-processing on map object for efficency
        self.walls = np.zeros((2*len(self.obsticles), 2))
        self.wallh = np.zeros((2*len(self.obsticles), 2))
        self.wallv = np.zeros((2*len(self.obsticles), 2))
        self.planeh = np.zeros(len(self.obsticles))
        self.planev = np.zeros(len(self.obsticles))
        self.limh = np.zeros((len(self.obsticles), 2))
        self.limv = np.zeros((len(self.obsticles), 2))

        for n in range(0, len(self.obsticles)):
            # if wall is vertical
            if self.obsticles[n].start[0] == self.obsticles[n].end[0]:
                self.wallv[2*n] = np.array([self.obsticles[n].start[0], self.obsticles[n].start[1]])
                self.wallv[2*n+1] = np.array([self.obsticles[n].end[0], self.obsticles[n].end[1]])

                self.planev[n] = self.wallv[2*n][0]
                self.limv[n] = np.array([np.min([self.obsticles[n].start[1], self.obsticles[n].end[1]])-0.5, np.max([self.obsticles[n].start[1], self.obsticles[n].end[1]])+0.5])

            # if wall is horizontal
            if self.obsticles[n].start[1] == self.obsticles[n].end[1]:
                self.wallh[2*n] = np.array([self.obsticles[n].start[0], self.obsticles[n].start[1]])
                self.wallh[2*n+1] = np.array([self.obsticles[n].end[0], self.obsticles[n].end[1]])

                self.planeh[n] = self.wallh[2*n][1]
                self.limh[n] = np.array([np.min([self.obsticles[n].start[0], self.obsticles[n].end[0]])-0.5, np.max([self.obsticles[n].start[0], self.obsticles[n].end[0]])+0.5])

            self.walls[2*n] = np.array([self.obsticles[n].start[0], self.obsticles[n].start[1]])
            self.walls[2*n+1] = np.array([self.obsticles[n].end[0], self.obsticles[n].end[1]])

    def empty(self, dim_w, dim_h, origin = [0,0]):
        box = make_box(2*dim_h, 2*dim_w, origin); [self.obsticles.append(box.walls[x]) for x in range(0, len(box.walls))]
        
        self.dimensions = [2*dim_h, 2*dim_w]






        
# Swarm behaviours and avoidance below

def avoidance(agents, map):

    size = len(agents)
    # Compute vectors between agents and wall planes
    diffh = np.array([map.planeh-agents[n][1] for n in range(size)])
    diffv = np.array([map.planev-agents[n][0] for n in range(size)])
        
    # split agent positions into x and y arrays
    agentsx = agents.T[0]
    agentsy = agents.T[1]

    # Check intersection of agents with walls
    low = agentsx[:, np.newaxis] >= map.limh.T[0]
    up = agentsx[:, np.newaxis] <= map.limh.T[1]
    intmat = up*low

    # Compute force based vector and multiply by intersection matrix
    Fy = np.exp(-2*abs(diffh) + 5)
    Fy = Fy*diffh*intmat

    low = agentsy[:, np.newaxis] >= map.limv.T[0]
    up = agentsy[:, np.newaxis] <= map.limv.T[1]
    intmat = up*low

    Fx = np.exp(-2*abs(diffv) + 5)
    Fx = Fx*diffv*intmat

    # Sum the forces between every wall into one force.
    Fx = np.sum(Fx, axis=1)
    Fy = np.sum(Fy, axis=1)
    # Combine x and y force vectors
    F = np.array([[Fx[n], Fy[n]] for n in range(size)])
    return F


def potentialField_map(env):

    # Set granularity of field map
    granularity = 0.5

    x = np.arange(-75, 74.9, granularity)
    y = np.arange(-40, 39.9, granularity)
    positions = np.zeros((len(x)*len(y), 2))

    count = 0
    for k in y:
        for j in x:
            positions[count][0] = j 
            positions[count][1] = k
            count += 1

    size = len(positions)
    # Compute vectors between agents and wall planes
    
    diffh = np.array([env.planeh-positions[n][1] for n in range(size)])
    diffv = np.array([env.planev-positions[n][0] for n in range(size)])
    
    # split agent positions into x and y arrays
    agentsx = positions.T[0]
    agentsy = positions.T[1]

    # Check intersection of agents with walls
    low = agentsx[:, np.newaxis] >= env.limh.T[0]
    up = agentsx[:, np.newaxis] <= env.limh.T[1]
    intmat = up*low

    # For larger environments
    A = 10; B = 10
    # For smaller environments
    #A = 2; B = 5

    # Compute force based vector and multiply by intersection matrix
    Fy = np.exp(-A*np.abs(diffh) + B)*diffh*intmat
    
    low = agentsy[:, np.newaxis] >= env.limv.T[0]
    up = agentsy[:, np.newaxis] <= env.limv.T[1]
    intmat = up*low
    now = time.time()
    Fx = np.exp(-A*np.abs(diffv) + B)*diffv*intmat

    # Sum the forces between every wall into one force.
    Fx = np.sum(Fx, axis=1)
    Fy = np.sum(Fy, axis=1)
    # Combine x and y force vectors

    F = np.stack((Fx, Fy), axis = 1)

    return F, positions

def fieldmap_avoidance(swarm):

    F = np.zeros((swarm.size, 2))

    x = np.arange(-75,74.8,0.5)
    y = np.arange(-40,39.9,0.5)

    f = swarm.field.reshape((len(y),len(x),2))

    x = np.round(2*swarm.agents.T[0])/2
    y = np.round(2*swarm.agents.T[1])/2

    inx = np.round(2*(y+40))
    iny = np.round(2*(x+75))

    for n in range(swarm.size):
        # Take agent position and find position in grid
        F[n] = f[int(inx[n])][int(iny[n])]
    
    return F


def return_to_base(swarm, base, param, modify):

    from_base_before = cdist(swarm.agents, [base.pos])
    
    if swarm.verbose == True:
        if np.sum(modify)>0:
            print(np.sum(modify), ' returning to base')
    
    alpha = 0.01; beta = 5
 
    #noise = 0.5*param*np.random.randint(-beta, beta, (swarm.size))
    #swarm.headings += noise
    #print(swarm.headings)

    # Calculate new heading vector
    gx = 1*np.cos(swarm.headings)
    gy = 1*np.sin(swarm.headings)
    G = -np.array([[gx[n], gy[n]] for n in range(0, swarm.size)])

    # Agent avoidance
    R = 20; r = 2; A = 1; a = 20    
    
    a = np.zeros((swarm.size, 2))

    # mag = cdist(swarm.agents, swarm.agents)

    # # Compute vectors between agents
    # diff = swarm.agents[:,:,np.newaxis]-swarm.agents.T[np.newaxis,:,:] 

    # R = 5; r = 5
    # repel = R*r*np.exp(-mag/r)[:,np.newaxis,:]*diff/(swarm.size-1)    
    # repel = np.sum(repel, axis = 0).T

    B = np.zeros((swarm.size, 2))
    #B = beacon(swarm)
    A_normal = avoidance(swarm.agents, swarm.map)
    #A_working = avoidance(swarm.agents, boxes.map_nest)
    
    
    A = A_normal*(1 - swarm.task)[:,None] #+ A_working*swarm.task[:,None]
    #A_normal[swarm.task == 1] = A_working[swarm.task == 1]
        
    a += A + G + B 

    vecx = a.T[0]
    vecy = a.T[1]

    angles = np.arctan2(vecy, vecx)
    Wx = swarm.speed*np.cos(angles)
    Wy = swarm.speed*np.sin(angles)

    W = -np.stack((Wx, Wy), axis = 1)
    swarm.agents += W*modify
    
    
    from_base_after = cdist(swarm.agents, [base.pos])
    
    for n in range(0, len(swarm.agents)):
        if modify[n, 0] == 1:
            if from_base_before[n] < from_base_after[n]:
                if swarm.verbose == True:
                    print('agent', n, ' correcting direction')
                noise = 5*param*np.random.randint(-beta, beta)
                swarm.headings[n] += noise
                #print('agent ', n, ' likes this direction')

def dispersion(swarm, vector, param, noise, modify):

    R = param; r = 2; A = 1; a = 20

    # Compute euclidean distance between agents
    mag = cdist(swarm.agents, swarm.agents)

    # Compute vectors between agents
    diff = swarm.agents[:,:,np.newaxis]-swarm.agents.T[np.newaxis,:,:] 

    A = fieldmap_avoidance(swarm)
    #A = avoidance(swarm.agents, swarm.map)
    #B = beacon(swarm)
    
    a = R*r*np.exp(-mag/r)[:,np.newaxis,:]*diff/(swarm.size-1)  
    a = np.sum(a, axis = 0).T

    a += A - vector + noise
    
    vecx = a.T[0]
    vecy = a.T[1]
    angles = np.arctan2(vecy, vecx)
    Wx = swarm.speed*np.cos(angles)
    Wy = swarm.speed*np.sin(angles)

    #W = -np.array([[Wx[n], Wy[n]] for n in range(0, swarm.size)])
    W = -np.stack((Wx, Wy), axis = 1)
    swarm.agents += W*modify

def continuous_boundary(agents, map):

    # Check if agent passes wall bounds. i.e. does agent intersect with area.

    # If yes, which side has the agent passed through?

    # Mirror agent back around to opposite wall.

    # split agent positions into x and y arrays
    agentsx = agents.T[0]
    agentsy = agents.T[1]

    # Set boundary size relative to environment dimensions
    scale = 1

    # Check left and right boundaries
    right = agentsx >=  scale*(map.dimensions[1]/2)
    left = agentsx <= -scale*(map.dimensions[1]/2)

    agentsx += -scale*(map.dimensions[1])*right
    agentsx += scale*(map.dimensions[1])*left

    # Check top and bottom boundaries
    top = agentsy >=  scale*(map.dimensions[0]/2)
    bottom = agentsy <= -scale*(map.dimensions[0]/2)

    agentsy += -scale*(map.dimensions[0])*top
    agentsy += scale*(map.dimensions[0])*bottom

    agents = np.stack((agentsx, agentsy), axis = 1) 

    return agents



    

    


def aggregate(swarm, param, noise):
    
    R = param; r = 3.5; A = 6.5; a = 7.5

    #noise = np.random.uniform(-.1, .1, (swarm.size, 2))

    # Compute euclidean distance between agents
    mag = cdist(swarm.agents, swarm.agents)

    # Compute vectors between agents
    diff = swarm.agents[:,:,np.newaxis]-swarm.agents.T[np.newaxis,:,:] 

    Avoid = fieldmap_avoidance(swarm)
    #B = beacon(swarm)
    
    repel = R*r*np.exp(-mag/r)[:,np.newaxis,:]*diff/(swarm.size-1)  
    repel = np.sum(repel, axis = 0).T

    attract = A*a*np.exp(-mag/a)[:,np.newaxis,:]*diff/(swarm.size-1)    
    attract = np.sum(attract, axis = 0).T

    total = 0
    total += Avoid + noise + repel - attract
    
    vecx = total.T[0]
    vecy = total.T[1]
    angles = np.arctan2(vecy, vecx)
    Wx = swarm.speed*np.cos(angles)
    Wy = swarm.speed*np.sin(angles)

    #W = -np.array([[Wx[n], Wy[n]] for n in range(0, swarm.size)])
    W = -np.stack((Wx, Wy), axis = 1)
    swarm.agents += W 
    


def rotate(swarm, direction, param):

    noise = param*np.random.randint(direction[0], direction[1], swarm.size)
    swarm.headings += noise

    # Calculate new heading vector
    gx = 1*np.cos(swarm.headings)
    gy = 1*np.sin(swarm.headings)
    G = -np.array([[gx[n], gy[n]] for n in range(0, swarm.size)])

    # Agent avoidance
    R = 2; r = 2; A = 1; a = 20
    # Compute euclidean distance between agents
    # mag = cdist(swarm.agents, swarm.agents)
    # # Compute vectors between agents
    # diff = swarm.agents[:,:,np.newaxis]-swarm.agents.T[np.newaxis,:,:] 
    # a = R*r*np.exp(-mag/r)[:,np.newaxis,:]*diff/(swarm.size-1)    
    # a = np.sum(a, axis =0).T

    a = np.zeros((swarm.size,2))
    B = np.zeros((swarm.size, 2))
    #B = beacon(swarm)
    A = avoidance(swarm.agents, swarm.map)
    a += G + A + B

    vecx = a.T[0]
    vecy = a.T[1]

    angles = np.arctan2(vecy, vecx)
    Wx = swarm.speed*np.cos(angles)
    Wy = swarm.speed*np.sin(angles)
    
    W = -np.array([[Wx[n], Wy[n]] for n in range(0, swarm.size)])
    swarm.agents += W 


def random_walk(swarm, boxes, base, param, modify):


    
    alpha = 0.01; beta = 5

    noise = param*np.random.randint(-beta, beta, (swarm.size))
    swarm.headings += noise
    #print(swarm.headings)

    # Calculate new heading vector
    gx = 1*np.cos(swarm.headings)
    gy = 1*np.sin(swarm.headings)
    G = -np.array([[gx[n], gy[n]] for n in range(0, swarm.size)])

    # Agent avoidance
    R = 20; r = 2; A = 1; a = 20    
    
    a = np.zeros((swarm.size, 2))

    # mag = cdist(swarm.agents, swarm.agents)

    # # Compute vectors between agents
    # diff = swarm.agents[:,:,np.newaxis]-swarm.agents.T[np.newaxis,:,:] 

    # R = 5; r = 5
    # repel = R*r*np.exp(-mag/r)[:,np.newaxis,:]*diff/(swarm.size-1)    
    # repel = np.sum(repel, axis = 0).T

    B = np.zeros((swarm.size, 2))
    #B = beacon(swarm)
    A_normal = avoidance(swarm.agents, swarm.map)
    A_working = avoidance(swarm.agents, boxes.map_nest)
    
    
    A = A_normal*(1 - swarm.task)[:,None] + A_working*swarm.task[:,None]
    #A_normal[swarm.task == 1] = A_working[swarm.task == 1]
        
    a += A + G + B 

    vecx = a.T[0]
    vecy = a.T[1]

    angles = np.arctan2(vecy, vecx)
    Wx = swarm.speed*np.cos(angles)
    Wy = swarm.speed*np.sin(angles)

    W = -np.stack((Wx, Wy), axis = 1)
    swarm.agents += W*modify
    
    
    from_base = cdist(swarm.agents, [base.pos])
    
    for n in range(0, len(swarm.agents)):
        if modify[n, 0] == 1:
            if base.move_radius + 5 > from_base[n] > base.move_radius :
                swarm.agents[n,:] -= 4*W[n,:]
                #swarm.headings[n] = 5*param*np.random.randint(-beta, beta)
                swarm.headings[n] = swarm.headings[n] - np.pi
            if base.move_radius + 5 < from_base[n]:
                swarm.lost[n] = 1
                

def foraging(swarm, param):

    alpha = 0.01; beta = 50

    noise = param*np.random.randint(-beta, beta, (swarm.size))
    swarm.headings += noise

    # Calculate new heading vector
    gx = 1*np.cos(swarm.headings)
    gy = 1*np.sin(swarm.headings)
    G = -np.array([[gx[n], gy[n]] for n in range(0, swarm.size)])

    # Agent avoidance
    R = 20; r = 2; A = 1; a = 20    
    
    a = np.zeros((swarm.size, 2))

    # mag = cdist(swarm.agents, swarm.agents)

    # # Compute vectors between agents
    # diff = swarm.agents[:,:,np.newaxis]-swarm.agents.T[np.newaxis,:,:] 

    # R = 5; r = 5
    # repel = R*r*np.exp(-mag/r)[:,np.newaxis,:]*diff/(swarm.size-1)    
    # repel = np.sum(repel, axis = 0).T

    B = np.zeros((swarm.size, 2))
    #B = beacon(swarm)
    A = avoidance(swarm.agents, swarm.map)
    a += A + G + B 

    vecx = a.T[0]
    vecy = a.T[1]

    angles = np.arctan2(vecy, vecx)
    Wx = swarm.speed*np.cos(angles)
    Wy = swarm.speed*np.sin(angles)

    W = -np.stack((Wx, Wy), axis = 1)
    swarm.agents += W
