from utils import *

class collision_check:
    def __init__(self, blocks):
        self.block = []
        for b in blocks:
            self.block.append(b[:6])
        # print(self.block)
        self.bn = len(self.block)


    def check(self, segment):
        """ return True if there is collision
        segment: line segment with: ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax']
        """
        cli = [i for i in range(self.bn)]
        return self.checkdimdfs(segment, cli)


    def checkdimdfs(self, segment, cli, axis = 0):
        """
        dimension dfs to filter the blocks which will certainly not collide, and then check the remain blocks
        """
        if axis == 3:
            # print("the possible collision blocks are" + str(cli))
            a, b = linear_equation(segment[0],segment[3],segment[1],segment[4])
            c, d = linear_equation(segment[1],segment[4],segment[2],segment[5])
            for bindex in cli:
                if self.collision_cube_line(bindex, segment, [a,b,c,d]):
                    print("the collision happens where segment = " + str(segment) + ", and block index is:" + str(bindex))
                    return True
            return  False
        tonext = []
        for seg in cli:
            if self.intersection([self.block[seg][axis], self.block[seg][axis+3]], [segment[axis], segment[axis+3]]):
                tonext.append(seg)
        return self.checkdimdfs(segment, tonext, axis+1)

    # def check_cubic_block(self, cubic, cli, axis = 0):
    #     """
    #     dimension dfs to filter the blocks which will certainly not collide, and then check the remain blocks
    #     """
    #     if axis == 3:
    #         return len(cli) > 0
    #     tonext = []
    #     for seg in cli:
    #         if self.intersection([self.block[seg][axis], self.block[seg][axis+3]], [segment[axis], segment[axis+3]]):
    #             tonext.append(seg)
    #     return self.checkdimdfs(segment, tonext, axis+1)


    def intersection(self, a, b):
        a.sort()
        b.sort()
        if a[0]>= b[1] or a[1] <= b[0]:
            return False
        return [max(a[0],b[0]), min(a[1],b[1])]

    def truncation(self, twopoints, axis, orisegment):
        pass


    def collision_cube_line(self, cindex, segment, para):
        """check if a line is cross a cube
        """
        # y = ax + b; z = cy + d
        a,b,c,d = para
        x1,y1,z1,x2,y2,z2 = segment

        cube = self.block[cindex]
        commonx = self.intersection([cube[0], cube[3]], [segment[0], segment[3]])
        if not commonx:
            return False
        if a != float("inf"):  # if a is not inf
            y1 = a*commonx[0]+b
            y2 = a*commonx[1]+b
        commony = self.intersection([cube[1], cube[4]], [y1, y2])
        if not commony:
            return False
        if c != float("inf"):
            z1 = c*commony[0]+d
            z2 = c*commony[1]+d
        commonz = self.intersection([cube[2], cube[5]], [z1, z2])
        if commonz[1] - commonz[0] > 0.000001:
            # print(para)
            # print([x1,x2,y1,y2,z1,z2])
            print(cindex)
            return commonz
        else:
            return  False



if __name__  == "__main__":
    # blocks = [[1,1,1,2,2,2],[3,0,0,4.5,0.5,0.5],[0,0,0,0.5,0.5,0.5]]  # , ,
    # blocks = [[4.5, 4.5, 2.5, 5.5, 5.5, 3.5]]
    # colcheck = collision_check(blocks)
    # print(colcheck.intersection([1,1],[2,0]))

    # print(colcheck.check([5,0.1,0.1, 0.1,0.1,0.1]))
    # print(colcheck.check([4.320725942163691, 4.320725942163691, 3.320725942163691, 4.6094010767585045, 4.6094010767585045, 3.609401076758504]))

    blocks = [[0.0,  2.0,  1.45 , 3.0 , 2.5,  4.55]]
    colcheck = collision_check(blocks)
    print(colcheck.check([3.1, 1.9, 3.1, 3.1, 3.9, 3.1]))