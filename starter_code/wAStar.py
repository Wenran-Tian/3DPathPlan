import numpy as np
import time
import matplotlib.pyplot as plt; plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import Planner
from collsion_check import *
import heapq
from utils import *
from parameters import *
from main import *

class wAStar:
    def __init__(self, boundary, blocks, cell_density = 5, weight = 1):
        """
            cell_density: cells per unit length
            weight: the epsilon value for heuristic function
        """
        # we assume that boundary will be integer
        boundary = boundary[0][0:6]
        x = int(boundary[3]-boundary[0])
        y = int(boundary[4]-boundary[1])
        z = int(boundary[5]-boundary[2])
        self.boundary = np.array(boundary)
        self.mxi = x * cell_density - 1
        self.myi = y * cell_density - 1
        self.mzi = z * cell_density - 1
        self.space = np.zeros((x * cell_density, y * cell_density, z * cell_density), dtype=np.int)
        self.dist = 1 / cell_density
        self.blocks = blocks
        self.weight = weight
        self.insert_blocks()

    def insert_blocks(self):
        """
            fill the voxel where there is block exits
        """
        for block in self.blocks:
            # print(np.array([block[0], block[3]]))
            p1 = self.c2g(block[0:3])
            p2 = self.c2g(block[3:6])
            # print("block space:" + str(p1) + str(p2) )
            self.space[p1[0]:p2[0]+1, p1[1]:p2[1]+1, p1[2]:p2[2]+1] = 1

    def c2g(self, point):
        """
            continuous world coordinate to grid voxel index
        """
        point = point - self.boundary[0:3]
        res = np.ceil(point/self.dist)-1
        res[res<0] = 0
        # res.astype(np.int)
        res = np.array([int(res[0]), int(res[1]), int(res[2])])
        return res

    def g2c(self, index):
        """
            grid voxel index to continuous world coordinate
        """
        return (index)*self.dist + self.boundary[0:3] + 0.5*self.dist

    def h(self,point, goal):
        """
            h is admissible
        """
        delta = np.abs(point-goal)
        return max(delta) + 0.7*min(delta)

    def solve(self, start, goal, show_path = False):
        """
            the main operation of weighted A* algorithm
        """
        si = self.c2g(start)
        # print(si)
        ti = self.c2g(goal)
        closed = np.zeros(self.space.shape)
        open = [[0, si[0], si[1], si[2]]]
        open_buff = {hashindex(si):0}
        g_matrix = np.ones(self.space.shape)*np.inf
        g_matrix[si[0], si[1], si[2]] = 0

        p_matrix = [[[[0,0,0] for i in range(self.mzi+1)] for i in range(self.myi+1)] for i in range(self.mxi+1)]
        p_matrix[si[0]][si[1]][si[2]] = 0

        c = 0
        while closed[ti[0],ti[1],ti[2]] == 0:
            # c += 1
            # if c % 500 == 0:
            #     print("iteration times:" + str(c))
            # g, px,py,pz = open.pop(0)

            g, px, py, pz = heapq.heappop(open)
            if closed[px][py][pz] == 1:
                continue

            point = [px,py,pz]
            # open_buff.pop(hashindex(point))

            closed[point[0],point[1],point[2]] = 1
            if g_matrix[point[0],point[1],point[2]] == np.inf:
                break
            child_points, costs = self.findChild(point, closed)
            # print(child_points)
            for i, npoint in enumerate(child_points):
                nval = g_matrix[point[0],point[1],point[2]] + costs[i]
                if g_matrix[npoint[0],npoint[1],npoint[2]] > nval:
                    g_matrix[npoint[0], npoint[1], npoint[2]] = nval
                    # print(npoint[0])
                    pointer = p_matrix[npoint[0]][npoint[1]][npoint[2]]
                    pointer[0] = point[0]
                    pointer[1] = point[1]
                    pointer[2] = point[2]

                    # hasflag = False
                    # for i in range(len(open)):
                    #     if open[i][1] == npoint[0] and open[i][2] == npoint[1] and open[i][3] == npoint[2]:
                    #         open[i][0] = nval + self.weight*self.h(npoint,ti)
                    #         hasflag = True
                    #         break
                    # if not hasflag:
                    #     open.append([nval + self.weight*self.h(npoint,ti), npoint[0], npoint[1], npoint[2]])
                    # open.sort()

                    heapq.heappush(open,[nval + self.weight*self.h(npoint,ti), npoint[0], npoint[1], npoint[2]])


        path = [ti]
        pointer = p_matrix[ti[0]][ti[1]][ti[2]]
        while pointer != 0:
            path.append(pointer)
            pointer = p_matrix[pointer[0]][pointer[1]][pointer[2]]
        # path = path[::-1]
        # path.appe
        path = np.array(path)
        path = self.g2c(path)
        path = path[::-1]
        n = np.sum(closed)
        print("The number of considered nodes is: "+str(n))
        if(show_path):
            print("the path will be:")
            print(path)
        return path,g_matrix[ti[0],ti[1],ti[2]]*self.dist



    def findChild(self,point, closed):
        res = []
        costs = []
        for i,dir in enumerate(DIRS):
            npoint = point.copy()
            npoint[0] += dir[0]
            npoint[1] += dir[1]
            npoint[2] += dir[2]
            if npoint[0] >= 0 and npoint[0] <= self.mxi and npoint[1] >= 0 and npoint[1] <= self.myi and npoint[2] >= 0 and npoint[2] <= self.mzi:
                if closed[npoint[0],npoint[1],npoint[2]] == 0 and self.space[npoint[0],npoint[1],npoint[2]] == 0:
                    res.append(npoint)
                    costs.append(COSTS[i])
        return res, costs





if __name__ == "__main__":
    # boundary = [[ -5.,  -5.,  -5. , 10.,  10.,  10. ,120., 120., 120.]]
    # blocks = [[4.5, 4.5, 2.5, 5.5, 5.5, 3.5],[0,0,0,1,1,1]]
    # planner = wAStar(boundary, blocks)
    # # print(planner.g2c(np.ones(3)*0))
    # # print(planner.c2g(np.ones(3)*-4.8))
    # planner.insert_blocks()
    # # print(planner.c2g(np.ones(3) * 10))
    # print(planner.space[52,52,42])
    # print(max(np.ones(3)))




    print('Running single cube test...\n')
    start = np.array([2.3, 2.3, 1.3])
    goal = np.array([7.0, 7.0, 5.5])
    success, pathlength = runtest('./maps/single_cube.txt', start, goal, True)
    print('Success: %r' % success)
    print('Path length: %d' % pathlength)
    print('\n')