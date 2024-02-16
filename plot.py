import numpy as np
import matplotlib.pyplot as plt
from numpy import e 
import math
import time
import pandas as pd
import os
#global position matrix
#global current velocity matrix
data_Array = np.zeros((7,13))

def reset():
    os.remove('time.csv')
    os.remove('priority.csv')
    os.remove('avg.csv')
if os.path.isfile('time.csv'):  
    reset()
#seed = 34
start=time.time()
i=0
from hyperparameters import noise, radius, delt, pert, b, sensor_range, n, maxv, attractive_gain, repulsive_gain, collision_distance, clipping_power, seed
np.random.seed(seed)
print("random seed:",seed)
"""
to calculate velocity:

F_total = sum(all Fs)

a = f/m = f

v = u + a*delt

clip v in magnitude

u -> old velocity table
v -> new velocity table

after calculating v, put u = v


pos_new = pos_old + v*delt

after calculating put pos_old = pos_new


-------
can reduce steps by
-------

maintain 'a' table, recalculated at every iteration

a = f(pos, priority)

once calculated for all UAVs

u = u + a*delt

clip u

pos = pos + u*delt
"""

#delt = 0.01
#radius = 100

#n=16

"""

function for initial positions and goals of n UAVs.
Equally space around a circle initially.
Goal = diammetrically opposite point + normal(0,pi/3)

example:
    n = 3
    start: [(0,0), (cos(2pi/3),sin(2pi/3)), (cos(4pi/3), sin(4pi/3))]
    goal: [(0,0), (-cos(2pi/3 + noise),-sin(2pi/3 + noise)), (-cos(4pi/3 + noise), -sin(4pi/3 + noise))]

    start:  [[10.          0.        ]
            [-5.          8.66025404]
            [-5.         -8.66025404]]
    
    goal    [[-9.73431938 -2.28976552]
            [ 3.43231798 -9.3925073 ]
            [-6.81016609  7.32267969]]
"""
from hyperparameters import spread, noise
def initialise_pos(n):
    start = []
    goal = []
    start_color = []
    goal_color = []


    for i in range(1,n//4+1):
        one = (np.cos((spread )*(i/(n//4))), np.sin((spread )*(i/(n//4))))
        two = (-np.cos((spread )*(i/(n//4))), np.sin((spread )*(i/(n//4))))
        three = (-np.cos((spread )*(i/(n//4))), -np.sin((spread )*(i/(n//4))))
        four = (np.cos((spread )*(i/(n//4))), -np.sin((spread )*(i/(n//4))))
        
        start += [one, two, three, four]
        goal += [two, one, four, three]

    cmap = plt.get_cmap('hsv')
    colors = [cmap(i) for i in np.linspace(0, 1, n+2)]
    colors = colors[2:]
    
    
    fig, ax = plt.subplots(figsize=(20,10))
    plt.scatter(radius*np.array(start)[:,0], radius*np.array(start)[:,1], c=colors, s=200)
    plt.scatter(1.2*radius*np.array(goal)[:,0], 1.2*radius*np.array(goal)[:,1], c=colors, marker="s", s=200, alpha=0.5)

    plt.xlim(-1.2*radius, 1.2*radius)
    plt.ylim(-radius/2,radius/2)
    #plt.ylim(-20,20)

    plt.gca().add_patch(plt.Circle((0,0), radius, fill=False))
    plt.gca().add_patch(plt.Circle((0,0), 1.2*radius, fill=False))
    plt.xlabel('x', fontsize=24)
    plt.ylabel('y', fontsize=24)
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)

    plt.savefig('/media/storage/Agam/pca/plots/comparison plots/start.eps',format='eps')
        

    
    start, goal = radius*np.array(start), 1.2*radius*np.array(goal)

    return start, goal

    start = []
    goal = []

    np.random.seed(3)
    for i in range(n):
        x = np.cos(2*i*np.pi/n)
        y = np.sin(2*i*np.pi/n)

        start.append((x, y))

        #noise = 0
        #noise = np.random.normal(0, np.pi/12)
        x_ = -np.cos(2*i*np.pi/n + noise)
        y_ = -np.sin(2*i*np.pi/n + noise)

        goal.append((x_, y_))
    
    start, goal = radius*np.array(start), radius*np.array(goal)

    cmap = plt.get_cmap('hsv')
    colors = [cmap(i) for i in np.linspace(0, 1, n)]
    #colors = colors[2:]
    
    
    fig, ax = plt.subplots(figsize=(20,10))
    plt.scatter(radius*np.array(start)[:,0], radius*np.array(start)[:,1], c=colors, s=200)
    plt.scatter(radius*np.array(goal)[:,0], radius*np.array(goal)[:,1], c=colors, marker="s", s=200, alpha=0.5)

    plt.xlim(radius, radius)
    plt.ylim(-radius,radius)

    plt.gca().add_patch(plt.Circle((0,0), radius, fill=False))
    plt.gca().add_patch(plt.Circle((0,0), 1.2*radius, fill=False))
    plt.xlabel('x', fontsize=18)
    plt.ylabel('y', fontsize=18)

    plt.savefig('/media/storage/Agam/pca/plots/experiments/5_uavs.eps',format='eps')

    return start, goal







"""
generator function for creating global position, velocity(u, v), priority, goal, completed tables from given n
"""

def generator(n):
    u = np.zeros((n,2))
    v = np.zeros((n,2))
    vmax = maxv*np.ones(n)
    

    pos, goal = initialise_pos(n)
    #priority = np.ones(n)


    a = np.zeros((n,2))

    completed = np.zeros(n)
    clip = np.zeros(n)

    return u, v, pos, goal, a, completed, clip, vmax

#n = 22 #no. of UAVs

u, v, pos, goal, a, completed, clip, vmax = generator(n)

from hyperparameters import priority_type
if priority_type=="Gaussian":
    priority = np.random.normal(3,1, n)
elif priority_type == "Uniform":
    priority = np.random.uniform(1,6, n)
else:
    priority = 3*np.ones(n)

#priority=[1,3,1,1]
in_collision = [[i] for i in priority]

#in_collision = [[i] for i in priority]

completion = [0]*n
priority_file = open("priority.csv",'a')
priority_file.write(", ".join([str(x) for x in priority])+'\n')
priority_file.close()

# exec_file = open('exec.csv', 'a')
"""
Now we calculate the forces to be applied

attractive: a(1-e**(-bd^2))[unit dir vector]  a:max acceleration, b:reducing spread, d:distance to goal

repulsive: ae**(-bd**2)[unit dir vector]  a:max repulsive force, b:repulsive spread, d:distance to obstacle
"""
check = np.ones(n)
min_dist = np.inf*np.ones((n,n))
avg_dist = np.zeros(n)

"""
two definitions for reached goal(completed trip), one checks wether distance to goal is less than 0.5

"""
def reached_goal(i):
    global pos, goal, r
    if np.linalg.norm(pos[i]-goal[i]) <= radius/100:
        #print(f"UAV {i} reached goal. Priority:{priority[i]}")
        return True
    else:
        return False
    # if np.linalg.norm(pos[i]) > r:
    #     return True
    

def collision(i,j):#return true if collision eminenent between j and i. collision threshold = 0.1
    
    t = -np.dot(pos[i]-pos[j],v[i]-v[j])/(np.dot(v[i]-v[j], v[i]-v[j])+0.00001)

    if t<0:
        return False
    else:
        s_quared = np.dot(pos[i] + v[i]*t - pos[j] - v[j]*t, pos[i] + v[i]*t - pos[j] - v[j]*t) 
        dist = np.linalg.norm(pos[i]-pos[j])

        if np.sqrt(s_quared)<collision_distance and dist < sensor_range:
            in_collision[i].append(priority[j])
            return True
        else:
            return False


def clip_v(n):
    global v, clip
    #print("----------------")
    for i in range(n):
        
        if clip[i]:
            vmax[i] = maxv*(priority[i]/max(in_collision[i]))**clipping_power
        else:
            vmax[i] = maxv
        #vmax[i]=maxv
        if np.linalg.norm(v[i])>vmax[i]:
            v[i] = vmax[i]*v[i]/np.linalg.norm(v[i])

        
        #print(i,v[i], np.linalg.norm(v[i]), vmax[i])


"""
Function to pertube angle of vector by a slight amount randomly. draw from N(0, pi/64)
let's pertube the whole v matrix
"""


def pertube(v):
    
    for i, vel in enumerate(v):
        x = vel[0]
        y = vel[1]

        delta_theta = np.random.normal(0,np.pi/2**10.5)
        theta = np.arctan2(y, x)

        # Perturb the angle by a small amount
        theta_perturbed = theta + delta_theta

        # Calculate the perturbed vector components using the perturbed angle
        x_perturbed = np.cos(theta_perturbed) * np.sqrt(x**2 + y**2)
        y_perturbed = np.sin(theta_perturbed) * np.sqrt(x**2 + y**2)

        v[i] = np.array([x_perturbed, y_perturbed])

"""
test:
v - pertube(v) != 0
"""

"""
rotate vector v=x,y by angle theta
"""

def rotate_vector(v, theta):
    x, y = v[0], v[1]
    # convert theta to radians
    #theta = math.radians(theta)
    
    # calculate sine and cosine of theta
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    
    # apply rotation formula
    x_new = x * cos_theta - y * sin_theta
    y_new = x * sin_theta + y * cos_theta
    
    return np.array([x_new, y_new])

r=0

def run_exp(number_of_uavs, iteration_number):


if __name__ == "__main__":

    file = open('out.csv','w')
    velocity_file = open('vel.csv', 'w')
    time_file = open('time.csv', 'a')
    acc_file = open('acc.csv', 'w')
    header = ""
    for m in range(n):
        header += str(m)+"x,"+str(m)+"y,"
    file.write(header+"\n")
    velocity_file.write(header+"\n")
    acc_file.write(header+"\n")
    header2 = ", ".join([f"{i+1}" for i in range(n)])+"\n"
    #time_file.write(", ".join([str(x) for x in priority]))

    while not np.array_equal(completed, check):
        # i+=1
        r+=1
        clip = np.zeros(n)
        a = np.zeros((n,2))
        for i in range(n):
            if reached_goal(i):
                if completed[i] != 1:
                    completed[i] = 1
                    a[i] = 0
                    v[i] = 0
                    completion[i]=r
                
            else:
                #attractive force 
                dist_to_goal = np.linalg.norm(goal[i]-pos[i])
                #attr = 2*(1-e**(-2*dist_to_goal**2))*np.array(goal[i]-pos[i])/dist_to_goal
                attr = attractive_gain*np.array(goal[i]-pos[i])/dist_to_goal
                a[i] = np.array(attr)
                #print("a",i,a[i])

                #repulsive potential
                colliding=False
                for j in range(n):
                    dist = np.linalg.norm(pos[j]-pos[i])
                    if dist < min_dist[i][j]:
                        min_dist[i][j] = dist
                    if j != i:
                        if collision(i, j):
                            colliding=True
                            #print("collision",i,j)
                            #dist = np.linalg.norm(pos[j]-pos[i])
                            rep = (priority[j]/priority[i])*repulsive_gain*(e**(-b*(dist-collision_distance)**2))*(pos[i]-pos[j])/dist
                            rep = rotate_vector(rep, np.pi/2)
                            #print(i,j,dist,rep,a[i])
                            a[i] += rep
                            
                
                if colliding:
                    clip[i]=1
        if pert:
            pertube(v)
        v = v + a*delt
        clip_v(n)
        pos = pos + v*delt

        file.write(",".join([str(x) for x in pos.flatten()])+"\n")
        velocity_file.write(", ".join([str(x) for x in v.flatten()])+"\n")
        acc_file.write(", ".join([str(x) for x in a.flatten()])+"\n")
    print(completed)
    print(check)
    time_file.write(", ".join([str(x) for x in completion])+'\n')
    time_file.close()

    file.close()
    velocity_file.close() 


    acc_file.close()

    for i in range(n):
        avg_dist[i] = np.mean(min_dist[i])
    
    avg_file = open('avg.csv','a')
    avg_file.write(",".join([str(x) for x in avg_dist])+"\n")
    # end = time.time()
    # excec_time = end-start
    # exec_file.write(f'{i},{excec_time}')
    print("n:",n, "priority: ",priority)
    avg_file.close()





                
            
        






    

