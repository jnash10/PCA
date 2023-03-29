import numpy as np
import matplotlib.pyplot as plt

pos=np.array([[1,0],[10,0]])

goal=np.array([[10,2],[0,2]])

v=np.array([[0,0],[0,0]])

vmax=5

delt=0.01

outfile = open('outfile.csv','w')
outfile.write("x1,y1,x2,y2\n")

def go_to_goal():
    global pos, goal, v
    for i in range(2):
        v[i] = vmax*(goal[i]-pos[i])/np.linalg.norm(goal[i]-pos[i])
    pos = pos + delt*v




for k in range(10000):
    out = f"{pos[0][0]},{pos[0][1]},{pos[1][0]},{pos[1][1]}\n"
    outfile.write(out)
    go_to_goal()



