from rrt_algorithms_develop.src.rrt.rrt_star import *
from rrt_algorithms_develop.src.search_space.search_space import *
import numpy as np
class RRTS:
    def __init__(self, boundary, blocks, samples = 163840, r = 0.01, prc = 0.01, rewire_count = 64):
        """
            sealing the RRT* alg from rrt_algorithms_develop.src.rrt.rrt_star, so that we can use it directly in main.py
        """
        boundary = boundary[0]
        nboundary = np.array([(boundary[0], boundary[3]),(boundary[1], boundary[4]),(boundary[2], boundary[5])])
        nblocks = []
        for block in blocks:
            nblocks.append(tuple(block[:6]))
        nblocks = np.array(nblocks)
        # if any(len(o) / 2 != 3 for o in O):
        #     raise Exception("Obstacle has incorrect dimension definition")
        print(nblocks)
        space = SearchSpace(nboundary, nblocks)
        self.space = space
        self.samples = samples
        self.r = r
        self.prc = prc
        self.rc = rewire_count

    def solve(self, start, goal):
        # print(tuple(start))
        # set the hyper-parameters here r, prc=0.01, rewire_count=None
        self.solver = RRTStar(self.space, np.array([(1, 10)]), tuple(start), tuple(goal), self.samples, self.r, self.prc, self.rc)
        path = self.solver.rrt_star()

        path_buff = np.array(list(path[0]))
        path_buff.reshape((1,3))
        for i in range(1,len(path)):
            path_buff = np.row_stack((path_buff, np.array(list(path[i]))))
        # print("the path will be:")
        # print(path_buff)

        delta = path_buff[1:] - path_buff[:-1]
        cost = np.sum(np.linalg.norm(delta,axis=1))

        return path,cost


if __name__ == "main":
    print("hw")