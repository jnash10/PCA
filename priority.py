import numpy as np
import matplotlib.pyplot as plt
from numpy import e 


#global position matrix
#global current velocity matrix


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

delt = 0.001
radius = 10
maxv = 10



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
def initialise_pos(n):
    start = []
    goal = []

    np.random.seed(5)
    for i in range(n):
        x = np.cos(2*i*np.pi/n)
        y = np.sin(2*i*np.pi/n)

        start.append((x, y))

        noise = 0
        #noise = np.random.normal(0, np.pi/6)
        x_ = -np.cos(2*i*np.pi/n + noise)
        y_ = -np.sin(2*i*np.pi/n + noise)

        goal.append((x_, y_))
    
    start, goal = radius*np.array(start), radius*np.array(goal)

    return start, goal







"""
generator function for creating global position, velocity(u, v), priority, goal, completed tables from given n
"""

def generator(n):
    u = np.zeros((n,2))
    v = np.zeros((n,2))
    vmax = maxv*np.ones(n)
    

    pos, goal = initialise_pos(n)
    priority = np.ones(n)


    a = np.zeros((n,2))

    completed = np.zeros(n)
    clip = np.zeros(n)

    return u, v, pos, goal, priority, a, completed, clip, vmax

n = 5 #no. of UAVs

u, v, pos, goal, priority, a, completed, clip, vmax = generator(n)

priority = np.array([1,2,3,2,1])

"""
Now we calculate the forces to be applied

attractive: a(1-e**(-bd^2))[unit dir vector]  a:max acceleration, b:reducing spread, d:distance to goal

repulsive: ae**(-bd**2)[unit dir vector]  a:max repulsive force, b:repulsive spread, d:distance to obstacle
"""
check = np.ones(n)


def reached_goal(i):
    global pos, goal
    if np.linalg.norm(pos[i]-goal[i]) <= 0.5:
        return True
    else:
        return False

def collision(i,j):#return true if collision eminenent between j and i. collision threshold = 0.1
    t = -np.dot(pos[i]-pos[j],v[i]-v[j])/(np.dot(v[i]-v[j], v[i]-v[j])+0.00001)

    if t<0:
        return False
    else:
        s_quared = np.dot(pos[i] + v[i]*t - pos[j] - v[j]*t, pos[i] + v[i]*t - pos[j] - v[j]*t) 

        if np.sqrt(s_quared)<0.1:
            return True
        else:
            return False


def clip_v(n):
    global v, clip
    print("----------------")
    for i in range(n):
        
        if clip[i]:
            vmax[i] = maxv*priority[i]/max(priority)
        else:
            vmax[i] = maxv

        if np.linalg.norm(v[i])>vmax[i]:
            v[i] = vmax[i]*v[i]/np.linalg.norm(v[i])
        print(i,v[i])

            

file = open('out.csv','w')
velocity_file = open('vel.csv', 'w')
acc_file = open('acc.csv', 'w')
header = ""
for m in range(n):
    header += str(m)+"x,"+str(m)+"y,"
file.write(header+"\n")
velocity_file.write(header+"\n")
acc_file.write(header+"\n")

while (completed != check).all():
    """
    for each UAV:
        if reached goal:
            completed[self] = 1

        attractive    
        calculate goal a
        add to a

        repulsive
        for each other UAV:
            if collision:
                clip[self] = True
                calculate repulsive a
                add to a
            else:
                pass

    
    v = v + a*delt
    clip v
    to clip v, 
    for each uav:
        if clip[i] == True
            vmax[i] = maxv*my_p/(max_p - lowest_p + 1)
        else:
            vmax[i] = maxv
        if v>vmax:
            v = maxvv[i]-v[j]
    

    completition check: within 0.1 of goal
    """
    clip = np.zeros(n)
    a = np.zeros((n,2))
    for i in range(n):
        if reached_goal(i):
            completed[i] = 1
            a[i] = 0
            v[i] = 0
            continue

        #attractive potential 
        dist_to_goal = np.linalg.norm(goal-pos)
        attr = 4*(1-e**(-2*dist_to_goal**2))*np.array(goal[i]-pos[i])/dist_to_goal
        a[i] = np.array(attr)
        #print("a",i,a[i])

        #repulsive potential
        colliding=False
        for j in range(n):
            if j != i:
                if collision(i, j):
                    colliding=True
                    print("collision",i,j)
                    dist = np.linalg.norm(pos[j]-pos[i])
                    rep = (priority[j]/priority[i])*10*(e**(-0.2*dist**2))*(pos[i]-pos[j])/dist
                    #print(i,j,dist,rep,a[i])
                    a[i] += rep
                    
        
        if colliding:
            clip[i]=1


    v = v + a*delt
    clip_v(n)
    pos = pos + v*delt

    file.write(",".join([str(x) for x in pos.flatten()])+"\n")
    velocity_file.write(", ".join([str(x) for x in v.flatten()])+"\n")
    acc_file.write(", ".join([str(x) for x in a.flatten()])+"\n")

file.close()
velocity_file.close() 


acc_file.close()



        
        
    






    

