import math
dirs = []
costs = []


def dfs(level,li):
    if level==3:
        dirs.append(li.copy())
        costs.append(math.sqrt(li[0]**2 + li[1]**2 + li[2]**2))
        return
    li.append(0)
    dfs(level+1,li)
    li.pop(-1)
    li.append(1)
    dfs(level+1,li)
    li.pop(-1)
    li.append(-1)
    dfs(level+1,li)
    li.pop(-1)

dfs(0,[])
DIRS = dirs[1:]
COSTS = costs[1:]