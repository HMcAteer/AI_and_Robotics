import queue


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
    searchType = input("Please select 1 for breadth first search or 2 for depth first search")
    choice = input(
        "Please type 1 if you would like to enter your own start and goal points, or type 2 if you would like to use "
        "[1,1] as start and [3,3] as goal")
    if choice is '1':
        startRow = int(input("please enter the row number for the start position"))
        startColumn = int(input("Please enter the column number for the start position"))
        goalRow = int(input("please enter the row number for the goal position"))
        goalColumn = int(input("Please enter the column number for the goal position"))

        start = [startRow, startColumn]
        goal = [goalRow, goalColumn]

    else:
        start = [1, 1]
        goal = [3, 3]
    uninformedSearch(grid, start, goal, searchType)


class Node:

    def __init__(self, value, parent):
        self.value = value
        self.parent = parent


def uninformedSearch(grid, start, goal, searchType):
    if searchType is '1':
        current = Node(start, '')
        openList = queue.Queue()
        openList.put(current)
        closedList = []
        closedValues = []
        while (current.value is not goal) and openList:
            current = openList.get()
            if current.value != goal:
                closedValues.append(current.value)
                closedList.append(current)
                expandNode(current, grid, openList, closedList, closedValues)
            else:
                break
        path = setPath(current)
        outputGrid(grid, start, goal, path)
    elif searchType is '2':
        current = Node(start, '')
        openList = queue.LifoQueue()
        openList._put(current)
        closedList = []
        closedValues = []
        while (current.value is not goal):
            current = openList.get()
            if current.value != goal and openList:
                closedValues.append(current.value)
                closedList.append(current)
                expandNode(current, grid, openList, closedList, closedValues)
            else:
                break
        path = setPath(current)
        outputGrid(grid, start, goal, path)


def getNeighbors(location, grid):
    row = location[0]
    column = location[1]
    neighbors = []

    if grid[row - 1][column] is 0:  # up
        neighbors.append([row - 1, column])
    if grid[row][column + 1] is 0:  # right
        neighbors.append([row, column + 1])
    if grid[row + 1][column] is 0:  # down
        neighbors.append([row + 1, column])
    if grid[row][column - 1] is 0:  # left
        neighbors.append([row, column - 1])
    return neighbors


def expandNode(current, grid, openList, closedList, closedValues):
    neighbors = getNeighbors(current.value, grid)

    for n in neighbors:
        node = Node([n[0], n[1]], current)
        if node not in closedList and node.value not in closedValues:
            openList.put(node)


def setPath(current):
    path = [current.value]
    while current.parent is not '':
        current = current.parent
        path.append(current.value)
    return path


if __name__ == '__main__':
    main()
