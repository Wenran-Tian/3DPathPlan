def linear_equation(x1,x2,y1,y2):
    """
        return a,b, where y = ax+b
    """
    if x1 == x2:
        return float("inf"),float("inf")
    a = (y1-y2)/(x1-x2)
    b = y1-a*x1
    return a,b


def hashindex(index):
    """
        hash a 3D coordinate to an integer
    """
    res = index[0] + index[1] * 10000 + index[2] * 100000000
    return res

def update_list(open):
    """
        update the open list, but not used in real implementation
    """
    open.sort()

    nopen_buff = {}
    for i in range(len(open)):
        nopen_buff[hashindex(open[i][1:])] = i
    return  nopen_buff


if __name__ == "__main__":
    open_buff = {}
    open = [[1.0, 36, 36, 32]]
    open_buff = update_list(open)

    print(open)
    print(open_buff)
