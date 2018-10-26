import math
import time
import matplotlib.pyplot as plt
import turtle

# Checks if the edge is jumped over
def checkEdge(edge, dx, dy, currentNode, parentNode):
        if (edge == "left"):
                if (dx != 0):
                        xDist = currentNode[1] - parentNode[1] - 0.5
                        y1Dist = currentNode[0] - parentNode[0] - 0.5
                        y2Dist = currentNode[0] - parentNode[0] + 0.5
                        yPlane = currentNode[0] - parentNode[0]
                        if (yPlane == 0 and dx < 0):
                                if (abs(dy/dx) <= abs(y1Dist/xDist)):
                                        return True
                        else:
                                slope1 = abs(y1Dist/xDist)
                                slope2 = abs(y2Dist/xDist)
                                if (abs(dy/dx) >= min(slope1, slope2) and abs(dy/dx) <= max(slope1, slope2) and dx < 0):
                                        return True
        elif (edge == "right"):
                if (dx != 0):
                        xDist = currentNode[1] - parentNode[1] + 0.5
                        y1Dist = currentNode[0] - parentNode[0] - 0.5
                        y2Dist = currentNode[0] - parentNode[0] + 0.5
                        yPlane = currentNode[0] - parentNode[0]
                        if (yPlane == 0 and dx > 0):
                                if (abs(dy/dx) <= abs(y1Dist/xDist)):
                                        return True
                        else:
                                slope1 = abs(y1Dist/xDist)
                                slope2 = abs(y2Dist/xDist)
                                if (abs(dy/dx) >= min(slope1, slope2) and abs(dy/dx) <= max(slope1, slope2) and dx > 0):
                                        return True
        elif (edge == "top"):
                if (dx != 0):
                        yDist = currentNode[0] - parentNode[0] - 0.5
                        x1Dist = currentNode[1] - parentNode[1] - 0.5
                        x2Dist = currentNode[1] - parentNode[1] + 0.5
                        xPlane = currentNode[1] - parentNode[1]
                        if (xPlane == 0 and dy < 0):
                                if (abs(dy/dx) >= abs(yDist/x1Dist)):
                                        return True
                        else:
                                slope1 = abs(yDist/x1Dist)
                                slope2 = abs(yDist/x2Dist)
                                if (abs(dy/dx) >= min(slope1, slope2) and abs(dy/dx) <= max(slope1, slope2) and dy < 0):
                                        return True
                elif (dy < 0):
                        return True
        elif (edge == "bottom"):
                if (dx != 0):
                        yDist = currentNode[0] - parentNode[0] + 0.5
                        x1Dist = currentNode[1] - parentNode[1] - 0.5
                        x2Dist = currentNode[1] - parentNode[1] + 0.5
                        xPlane = currentNode[1] - parentNode[1]
                        if (xPlane == 0 and dy > 0):
                                if (abs(dy/dx) >= abs(yDist/x1Dist)):
                                        return True
                        else:                       
                                slope1 = abs(yDist/x1Dist)
                                slope2 = abs(yDist/x2Dist)
                                if (abs(dy/dx) >= min(slope1, slope2) and abs(dy/dx) <= max(slope1, slope2) and dy > 0):
                                        return True
                elif (dy > 0):
                        return True
        return False

# Finds the collision from the parent block to target block
def isCollision(parentNode, targetNode, g, speed, width, buildingHeights, vy):
        crashed = False
        vx = math.sqrt((speed*speed) - (vy*vy))
        dx = targetNode[1] - parentNode[1]   # 'horizontal' distance covered in a 2D block configuration
        dy = targetNode[0] - parentNode[0]   # 'vertical' distance covered in a 2D block configuration

        # variables storing detection values for each edge of a given block
        impactLeft = False
        impactRight = False
        impactTop = False
        impactBottom = False
        currentNode = [parentNode[0], parentNode[1]]    # start at the parent block
        
        while (currentNode[0] != targetNode[0] or currentNode[1] != targetNode[1]): # as long as we havent reached the end

                # check each edge of the current block
                impactLeft = checkEdge("left", dx, dy, currentNode, parentNode)
                impactRight = checkEdge("right", dx, dy, currentNode, parentNode)
                impactTop = checkEdge("top", dx, dy, currentNode, parentNode)
                impactBottom = checkEdge("bottom", dx, dy, currentNode, parentNode)

                # the following four variables contain detection values for jumps made along corners
                topLeft = impactLeft and impactTop
                topRight = impactRight and impactTop
                bottomLeft = impactLeft and impactBottom
                bottomRight = impactRight and impactBottom

                if (topLeft == True):
                        xDist = currentNode[1] - parentNode[1] + 0.5
                        hDiff1 = buildingHeights[currentNode[0]][currentNode[1] - 1] - buildingHeights[parentNode[0]][parentNode[1]]
                        hDiff2 = buildingHeights[currentNode[0] - 1][currentNode[1]] - buildingHeights[parentNode[0]][parentNode[1]]
                        currentNode[0] -= 1
                        currentNode[1] -= 1
                        hDiff3 = buildingHeights[currentNode[0]][currentNode[1]] - buildingHeights[parentNode[0]][parentNode[1]]
                        height = findHeight(xDist, width, g, vy, vx, dx, dy)
                        crashed = (height < hDiff1 or height < hDiff2 or height < hDiff3)

                elif (topRight == True):
                        xDist = currentNode[1] - parentNode[1] + 0.5
                        hDiff1 = buildingHeights[currentNode[0]][currentNode[1] + 1] - buildingHeights[parentNode[0]][parentNode[1]]
                        hDiff2 = buildingHeights[currentNode[0] - 1][currentNode[1]] - buildingHeights[parentNode[0]][parentNode[1]]
                        currentNode[0] -= 1
                        currentNode[1] += 1
                        hDiff3 = buildingHeights[currentNode[0]][currentNode[1]] - buildingHeights[parentNode[0]][parentNode[1]]
                        height = findHeight(xDist, width, g, vy, vx, dx, dy)
                        crashed = (height < hDiff1 or height < hDiff2 or height < hDiff3)

                elif (bottomLeft == True):
                        xDist = currentNode[1] - parentNode[1] + 0.5
                        hDiff1 = buildingHeights[currentNode[0]][currentNode[1] - 1] - buildingHeights[parentNode[0]][parentNode[1]]
                        hDiff2 = buildingHeights[currentNode[0] + 1][currentNode[1]] - buildingHeights[parentNode[0]][parentNode[1]]
                        currentNode[0] += 1
                        currentNode[1] -= 1
                        hDiff3 = buildingHeights[currentNode[0]][currentNode[1]] - buildingHeights[parentNode[0]][parentNode[1]]
                        height = findHeight(xDist, width, g, vy, vx, dx, dy)
                        crashed = (height < hDiff1 or height < hDiff2 or height < hDiff3)

                elif (bottomRight == True):
                        xDist = currentNode[1] - parentNode[1] + 0.5
                        hDiff1 = buildingHeights[currentNode[0]][currentNode[1] + 1] - buildingHeights[parentNode[0]][parentNode[1]]
                        hDiff2 = buildingHeights[currentNode[0] + 1][currentNode[1]] - buildingHeights[parentNode[0]][parentNode[1]]
                        currentNode[0] += 1
                        currentNode[1] += 1
                        hDiff3 = buildingHeights[currentNode[0]][currentNode[1]] - buildingHeights[parentNode[0]][parentNode[1]]
                        height = findHeight(xDist, width, g, vy, vx, dx, dy)
                        crashed = (height < hDiff1 or height < hDiff2 or height < hDiff3)

                elif (impactLeft == True):
                        xDist = currentNode[1] - parentNode[1] - 0.5
                        currentNode[1] -= 1
                        hDiff = buildingHeights[currentNode[0]][currentNode[1]] - buildingHeights[parentNode[0]][parentNode[1]]
                        height = findHeight(xDist, width, g, vy, vx, dx, dy)
                        crashed = (height < hDiff)

                elif (impactRight == True):
                        xDist = currentNode[1] - parentNode[1] + 0.5
                        currentNode[1] += 1
                        hDiff = buildingHeights[currentNode[0]][currentNode[1]] - buildingHeights[parentNode[0]][parentNode[1]]
                        height = findHeight(xDist, width, g, vy, vx, dx, dy)
                        crashed = (height < hDiff)

                elif (impactTop == True):                                      
                        yDist = currentNode[0] - parentNode[0] - 0.5
                        currentNode[0] -= 1
                        hDiff = buildingHeights[currentNode[0]][currentNode[1]] - buildingHeights[parentNode[0]][parentNode[1]]
                        height = findHeight(yDist, width, g, vy, vx, dy, dx)
                        crashed = (height < hDiff)

                elif (impactBottom == True):                                   
                        yDist = currentNode[0] - parentNode[0] + 0.5
                        currentNode[0] += 1
                        hDiff = buildingHeights[currentNode[0]][currentNode[1]] - buildingHeights[parentNode[0]][parentNode[1]]
                        height = findHeight(yDist, width, g, vy, vx, dy, dx)
                        crashed = (height < hDiff)
                
                if (crashed == True):
                        break
        return crashed

# Finds the height of a jump given distance and speed
def findHeight(xDist, width, g, vy, vx, dx, dy):
        yDist = 0
        # avoiding division by zero. If there is no x-component, the distance is equal to the y-component
        if (dx == 0):
                distance = yDist
        else:
                yDist = xDist * dy / dx
                distance = math.sqrt((width * width * xDist * xDist) + (width * width* yDist * yDist))
        time = distance / vx
        height = (vy * time) - (0.5 * g * time * time)
        return height

# Main algorithm for finding jumps. Also determines if there are collisions during jumps
def findJumps(width, speed, g, jumpGrid, buildingHeights, treeArray, tooHigh, depth):

        # base case i.e. no more blocks can be jumped over
        if(len(treeArray[depth]) == 0):
                return
        
        # find the tree depth
        depth = len(treeArray)
       
        # add another row for next group of jumps
        treeArray.append([])

        # takes each element in the treeArray (blocks which are to be jumped from) and 
        # tries jumping to each element in the tooHigh array (contains blocks not jumped to/over)
        for parent in treeArray[depth - 1]:
                temp = list(tooHigh)
                for node in temp:
                        vy = findVy(parent[0], parent[1], node[0], node[1], g, speed, width, buildingHeights)
                        # jump has sufficient vertical speed and clears all collisions along the path
                        if ((vy > 0) and (isCollision(parent, node, g, speed, width, buildingHeights, vy) == False)):
                                        jumpGrid[node[0]][node[1]] = depth   # depth denotes the number of jumps made at each level of the tree
                                        treeArray[depth].append(node)  # the target now becomes the starting point of another possible jump
                                        tooHigh.remove(node)   # remove the block since it is successfully jumped to

        # call the function recursively using the updated starting and remaining blocks
        findJumps(width, speed, g, jumpGrid, buildingHeights, treeArray, tooHigh, depth)
                        
# Finds the vertical speed required by a jump given certain height and distance
def findVy(x1, y1, x2, y2, g, v, w, bh):
        d = math.sqrt(pow(w*(x2 - x1), 2) + pow(w*(y2 - y1), 2))
        h = bh[x2][y2] - bh[x1][y1]

        a = (h*h) + (d*d)
        b = (-2*v*v*h*h) - (g*d*d*h) - (v*v*d*d)
        c = (v*v*v*v*h*h) + (g*d*d*v*v*h) + ((g*g*d*d*d*d)/4.0)
        conj = (b*b)-(4*a*c)

        if (conj < 0):
                return 0
        else:
                vysquared = (-b + math.sqrt(conj)) / (2*a)
                vy = math.sqrt(vysquared)
                return vy


def main():
        g = 9.81    
        jumpGrid = []  
        buildingHeights = []
        treeArray = []
        tooHigh = []

        print("Enter the six integers for dx, dy, w, v, lx, ly:")
        inputVariables = list(map(int, input().split()))    # inputVariables contains the list of input numbers

        startingCoords = (inputVariables[-2], inputVariables[-1])  # extracting the starting point
        gridArea = (inputVariables[1], inputVariables[0])     # extracting the total grid area

        width = inputVariables[2]
        speed = inputVariables[3]

        # prepare jump grid by adding 'X' to all entries
        for i in range (gridArea[0] + 1):
                jumpGrid.append([])
                for j in range (gridArea[1] + 1):
                        jumpGrid[i].append('X')

        # create list of blocks remaining to be jumped to
        for i in range (1, gridArea[0] + 1):
                for j in range (1, gridArea[1] + 1):
                        if ((i, j) == startingCoords):
                                continue
                        tooHigh.append((i, j))        

        # get building heights
        buildingHeights.append([0,0,0,0,0])
        print("Enter the building heights: ")
        for i in range (inputVariables[1]):
                inputList = list(map(int, input().split()))
                inputList.insert(0, 0)
                buildingHeights.append(inputList)

        # treeArray contains all the blocks from where jumps are initiated
        treeArray.append([])
        treeArray[0].append(startingCoords)
        jumpGrid[startingCoords[0]][startingCoords[1]] = 0
        
        # at start treeArray only has the starting block
        depth = 0

        startTime = time.time()

        # calls the jump finding algorithm
        findJumps(width, speed, g, jumpGrid, buildingHeights, treeArray, tooHigh, depth)
        elapsedTime = (time.time() - startTime) * 1000

        # once done, print the jump grid showing the number of jumps to each block
        print("Jump Grid:")
        for i in range (1, gridArea[0] + 1):
                for j in range(1, gridArea[1] + 1):
                        print(jumpGrid[i][j], end = ' ')
                print()
        print("Algorithm took ", round(elapsedTime, 4), " milli-seconds.")

        t = turtle.Turtle()
        w = turtle.Screen()
        t.penup()
        t.setposition(-200,200)
        for i in range(gridArea[0]):
                for j in range(gridArea[1]):
                        for k in range(4):
                                t.pendown()
                                t.forward(width)
                                t.right(90)
                        t.penup()
                        t.forward(width)
                t.penup()
                t.forward(-1*width*gridArea[1])
                t.right(90)
                t.forward(width)
                t.left(90)
        
        w.exitonclick()


# Main function that runs the program
main()
