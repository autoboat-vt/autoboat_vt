import math as m
import heapq as h
import turtle as t
from nodeSpace import Space
from node import Node
from random import randint
from checkpoint import Inside

# check bounds
def isValid(row, col):
    return (row >= 0) and (row < rows) and (col >= 0) and (col < cols)

# check if cell is not blocked
def isOpen(grid, row, col):
    return grid[row][col] == 1

# check if node is destination
def checkHit(row, col, dest):
    return row == dest[0] and col == dest[1]

# Calculate the heuristic value
def calch(row, col, dest):
    return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5

# tracing path
def tracer(nodeInfo, dest):
    print("The Path is ")
    path = []
    row = dest[0]
    col = dest[1]

    # Trace the path from destination to source using the cell information
    while not (nodeInfo[row][col].ipos == row and nodeInfo[row][col].jpos == col):
        path.append([row, col])
        # temporary moved and worked until start
        temp_row = nodeInfo[row][col].ipos
        temp_col = nodeInfo[row][col].jpos

        # these are local row and col for nodeinfo
        row = temp_row
        col = temp_col

    # Add the source cell to the path
    path.append([row, col])
    # Reverse the path to get the path from source to destination
    path.reverse()

    # Print the path
    for i in path:
        print("->", i, end=" ")
    print()

    return path

def astar(grid, src, dest):

    # out of map
    if not isValid(src[0], src[1]) or not isValid(dest[0], dest[1]):
        print("Source or destination is invalid")
        return

    # bound to fail
    if not isOpen(grid, src[0], src[1]) or not isOpen(grid, dest[0], dest[1]):
        print("Source or the destination is blocked")
        return
    
    # lucky go
    if checkHit(src[0], src[1], dest):
        print("We are already at the destination")
        return

    # making list to close nodes (prevent cyclial graph)
    closed = [[False for _ in range(cols)] for _ in range(rows)]

    # setting a node object to each position
    nodeInfo = [[Node() for _ in range(cols)] for _ in range(rows)]

    # initializing the starting node
    i = src[0]
    j = src[1]
    nodeInfo[i][j].f = 0
    nodeInfo[i][j].g = 0
    nodeInfo[i][j].h = 0
    nodeInfo[i][j].ipos = i
    nodeInfo[i][j].jpos = j

    # Initialize the open list (cells to be visited) with the start cell
    open_list = []
    h.heappush(open_list, (0.0, i, j))


    # ending condition
    found = False

    # the main part of the A* algorithm
    while len(open_list) > 0:
        # Pop cell coordinate with the smallest f value from the open list
        p = h.heappop(open_list)

        # Mark the cell as visited & add to closed
        i = p[1]
        j = p[2]
        closed[i][j] = True

        # For each 8 directions, check the successors
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)] 
        for dir in directions:
            new_i = i + dir[0]
            new_j = j + dir[1]
            # If the successor is valid, unblocked, and not visited
            if isValid(new_i, new_j) and isOpen(grid, new_i, new_j) and not closed[new_i][new_j]:
                # If the successor is the destination
                if checkHit(new_i, new_j, dest):
                    # Set the parent of the destination cell
                    nodeInfo[new_i][new_j].ipos = i
                    nodeInfo[new_i][new_j].jpos = j
                    print("The destination cell is found")

                    # Trace and print the path from source to destination
                    found_dest = True
                    return tracer(nodeInfo, dest)
                else:
                    # Calculate the new f, g, and h values
                    g_new = nodeInfo[i][j].g=1.2
                    h_new = calch(new_i, new_j, dest)
                    f_new = g_new + h_new

                    # If the cell is not in the open list or the new f value is smaller
                    if nodeInfo[new_i][new_j].f == float('inf') or nodeInfo[new_i][new_j].f > f_new:
                        # Add the cell to the open list
                        h.heappush(open_list, (f_new, new_i, new_j))
                        # Update the cell details
                        nodeInfo[new_i][new_j].f = f_new
                        nodeInfo[new_i][new_j].g = g_new
                        nodeInfo[new_i][new_j].h = h_new
                        nodeInfo[new_i][new_j].ipos = i
                        nodeInfo[new_i][new_j].jpos = j
    if not found:
        print("Failed to find the destination cell")

'''
# making a 20x20 grid of 1 and 0 randomized

grid = [[0 if randint(0,1) == 1 else 1 for _ in range(20)] for _ in range(20)]

grid[0][0]=1
grid[19][19]=1
print(grid)


'''

# 20 by 20 grid with polygon with block
grid = [[1 for _ in range(20)] for _ in range(20)]

polygon=[[2,2],[5,2],[5,5],[2,5]]

for m in range(20):
    for n in range(20):
        pt = Inside(m,n)
        print(pt.check(polygon))
        grid[m][n]=0 if pt.check(polygon) == True else 1

rows = len(grid)
cols = len(grid[0])
space = Space(grid)

space.poly(polygon)

src = [rows//2,cols//2]
dest = [19,19]
moveset = astar(grid, src, dest)

if not moveset:
    print("failed")
    space.draw_grid(src, dest)
    space.complete()
else:
    space.draw_grid(src, dest)
    space.trace(src, moveset)
    space.complete()