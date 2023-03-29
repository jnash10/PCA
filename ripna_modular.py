import numpy as np



n=4
vmax=5
delt=0.001
rdes=2

pos = []
vel = []
goal = []
counter=0

uavs = []


def t_go(id1,id2):
    num = np.dot(pos[id1]-pos[id2], vel[id1]-vel[id2])
    den = np.dot(vel[id1]-vel[id2],vel[id1]-vel[id2])+10**(-3)

    return -num/den

def zem(id1,id2):
    tgo = t_go(id1,id2)
    zemsq = np.dot(pos[id1]-pos[id2], pos[id1]-pos[id2]) + 2*np.dot(pos[id1]-pos[id2],vel[id1]-vel[id2])*vmax*tgo + np.dot(vel[id1]-vel[id2],vel[id1]-vel[id2])*(vmax*tgo)**2
    return np.sqrt(zemsq)







class UAV():
    def __init__(self, start, target): #initialise UAV by adding rows for record to pos, vel, goal
     
        global pos, vel, goal, counter
        pos.append(np.array([start[0],start[1]]))
        goal.append(np.array([target[0],target[1]]))
        vel.append(np.array([10**(-3),10**(-3)]))
        self.id=counter
        counter+=1

    def go_to_goal(self): #return velocity vector for go to goal
        heading = vmax*(goal[self.id]-pos[self.id])/np.linalg.norm(goal[self.id]-pos[self.id])
        vel[self.id]=heading

    def avoid_collision(self,id): #avoid collision of self with given id
        vel[self.id]=np.array([-vel[self.id][1],vel[self.id][0]])

    def move(self):
        candidates = []
        for i in range(n):
            if i != self.id:
                if zem(self.id,i) < rdes:
                    candidates.append(i)
        print(self.id, candidates)
        if candidates != []:
            times = []
            for id in candidates:
                times.append(t_go(self.id,id))
            
            min_time_id = candidates[times.index(min(times))]
            self.avoid_collision(min_time_id)
        else:
            self.go_to_goal()



for l in range(n):
    uav = UAV(10*np.array([np.cos(2*l*np.pi/n),np.sin(2*l*np.pi/n)]),-10*np.array([np.cos(2*l*np.pi/n),np.sin(2*l*np.pi/n)]))
    uavs.append(uav)

pos=np.array(pos)
goal=np.array(goal)
vel=np.array(vel)

outfile = open('outfile.csv','w')
out=""
for i in range(n):
    out = out+f"x{i},y{i},"
out+="\n"
outfile.write(out)


for k in range(10000):
    out=""
    for i in range(n):
        out+=f"{pos[i][0]},{pos[i][1]},"
    out+="\n"
    outfile.write(out)

    for uav in uavs:
        uav.move()
    pos = pos + vel*delt




outfile.close()
                    




