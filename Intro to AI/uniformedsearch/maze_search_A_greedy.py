import queue
import math


# The grid values must be separated by spaces, e.g.
# 1 1 1 1 1
# 1 0 0 0 1
# 1 0 0 0 1
# 1 1 1 1 1
# Returns a 2D list of 1s and 0s
def readGrid(filename):
    # print('In readGrid')
    grid = []
    with open(filename) as f:
        for l in f.readlines():
            grid.append([int(x) for x in l.split()])

    f.close()
    # print 'Exiting readGrid'
    return grid


# Writes a 2D list of 1s and 0s with spaces in between each character
# 1 1 1 1 1
# 1 0 0 0 1
# 1 0 0 0 1
# 1 1 1 1 1
def outputGrid(grid, start, goal, path):
    # print('In outputGrid')
    filenameStr = 'path.txt'

    # Open filename
    f = open(filenameStr, 'w')

    # Mark the start and goal points
    grid[start[0]][start[1]] = 'S'
    grid[goal[0]][goal[1]] = 'G'

    # Mark intermediate points with *
    for i, p in enumerate(path):
        if i > 0 and i < len(path) - 1:
            grid[p[0]][p[1]] = '*'

    # Write the grid to a file
    for r, row in enumerate(grid):
        for c, col in enumerate(row):

            # Don't add a ' ' at the end of a line
            if c < len(row) - 1:
                f.write(str(col) + ' ')
            else:
                f.write(str(col))

        # Don't add a '\n' after the last line
        if r < len(grid) - 1:
            f.write("\n")

    # Close file
    f.close()
    # print('Exiting outputGrid')


def main():
    grid = readGrid('grid.txt')
    searchType = int(input("Please select 1 for greedy search or 2 for A* search"))
    choice = int(input(
        "Please type 1 if you would like to enter your own start and goal points, or type 2 if you would like to use "
        "[1,1] as start and [3,3] as goal"))
    if choice == 1:
        startRow = int(input("please enter the row number for the start position"))
        startColumn = int(input("Please enter the column number for the start position"))
        goalRow = int(input("please enter the row number for the goal position"))
        goalColumn = int(input("Please enter the column number for the goal position"))

        start = [startRow, startColumn]
        goal = [goalRow, goalColumn]

    else:
        start = [1, 1]
        goal = [3, 3]
    informed_Search(grid, start, goal,searchType)


class Node:

    def __init__(self, value, parent):
        self.value = value
        self.parent = parent
        self.g = 0  # path cost
        self.h = 0  # heuristic cost
        self.f = self.g + self.h  # A*

    def __lt__(self, other):
        return self.value < other.value


def heuristicCost(node, goal):
    return math.sqrt(math.pow(goal[0] - node.value[0], 2) + math.pow(goal[1] - node.value[1], 2))


def informed_Search(grid, start, goal,choice):
    expandedNode = 0
    current = Node(start, '')
    openList = queue.PriorityQueue()
    current.h = heuristicCost(current, goal)
    current.g = grid[current.value[0]][current.value[1]]
    current.f = current.g + current.h
    if choice == 1:
        openList.put((current.h, current))#choice 1 does greedy search using the stored heuristic value
    elif choice == 2:
        openList.put((current.f, current))#choice 2 does A* search using the stored f value
    closedValues = []
    while (current.value is not goal) and openList:
        current = openList.get()[1]
        if current.value != goal:
            closedValues.append(current.value)
            print(current.value)
            expandNode(current, grid, openList, closedValues, goal,choice)
            expandedNode+=1

        else:
            break
    path = setPath(current)

    print("Path was found and printed to external file")
    outputGrid(grid, start, goal, path)
    print("Number of nodes expanded: ", expandedNode)
    path.reverse()
    print("Here is the locations that the path takes")
    print(path)
    print("The heuristic function used was the euclidean distance between the current node and the final goal")


def getNeighbors(location, grid):
    result = []

    up = location[:]
    up[0] -= 1
    if up[0] > -1 and grid[up[0]][up[1]] != 0:
        result.append(up)

    right = location[:]
    right[1] += 1
    if right[1] < len(grid[right[0]]) and grid[right[0]][right[1]] != 0:
        result.append(right)

    down = location[:]
    down[0] += 1
    if down[0] < len(grid) and grid[down[0]][down[1]] != 0:
        result.append(down)

    left = location[:]
    left[1] -= 1
    if left[1] > -1 and grid[left[0]][left[1]] != 0:
        result.append(left)
    return result


def expandNode(current, grid, openList, closedValues, goal, choice):
    neighbors = getNeighbors(current.value, grid)

    for n in neighbors:
        node = Node([n[0], n[1]], current)
        node.g = grid[node.value[0]][node.value[1]]
        node.h = heuristicCost(node, goal)
        node.f = node.g + node.h
        if node.value not in closedValues:
            if choice == 1:
                openList.put((node.h, node))
            elif choice == 2:
                openList.put((node.f, node))


def setPath(current):
    path = [current.value]
    while current.parent is not '':
        current = current.parent
        path.append(current.value)
    return path


if __name__ == '__main__':
    main()
