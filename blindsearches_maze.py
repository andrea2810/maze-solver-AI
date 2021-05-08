import time
import timeit
import sys
from collections import deque
import PIL
from PIL import Image
import numpy

#/********************GLOBAL VARIABLES********************/
GoalState = []
GoalNode = None #At finding solution
NodesExpanded = 0 #Total nodes visited
MaxSearchDeep = 0 #Max Deep
MaxFrontier = 0 #Max Frontier
Height = 0
Width = 0
#/**********************CLASS PIXEL**********************/
class Pixel:
    def __init__(self, y, x, parent, move, depth, cost, key):
        self.y = y
        self.x = x
        self.coordinates = [y, x]
        self.parent = parent
        self.move = move
        self.depth = depth
        self.cost = cost
        self.key = key

#/********************IS WHITE PIXEL*********************/
def isWhite(value):
    if value == 1:
        return True

#/********************GET ADJACENT*********************/
def getAdjacent(pixels, point):

    global NodesExpanded, Width, Height

    possibleMoves = ['Up', 'Down', 'Right', 'Left']
    nextPaths = []

    NodesExpanded += 1
    for movement in possibleMoves:
        x, y = move(point, movement)
        if x >= 0 and x < Width and y >= 0 and y < Height:
            if isWhite(pixels[y, x]):
                nextPaths.append(Pixel(
                    y,
                    x,
                    point,
                    movement,
                    point.depth + 1,
                    point.cost + 1,
                    0))
    return nextPaths

#/********************MOVE*********************/
def move(pixel, action):

    if action == 'Up':
        x = pixel.x
        y = pixel.y - 1
    elif action == 'Down':
        x = pixel.x
        y = pixel.y + 1
    elif action == 'Right':
        x = pixel.x + 1
        y = pixel.y
    elif action == 'Left':
        x = pixel.x - 1
        y = pixel.y

    return x, y

#/********************BFS*********************/
def bfs(start, end, pixels, im_paths):
    global MaxFrontier, GoalNode, MaxSearchDeep
    
    color_paths = [128, 128, 128]

    visited = set()

    Queue = deque([start])

    while Queue:
        node = Queue.popleft()
        arr = ','.join(str(x) for x in node.coordinates)
        visited.add(arr)
        im_paths[node.x, node.y] = (color_paths[0], 
                                    color_paths[1],
                                    color_paths[2])

        if node.x == end.x and node.y == end.y:
            GoalNode = node
            return im_paths
        possiblePaths = getAdjacent(pixels, node)
        for path in possiblePaths:
            path_map = ''
            for x in path.coordinates:
                path_map +=','.join(str(x))

            if path_map not in visited:
                Queue.append(path)
                visited.add(path_map)
                if path.depth > MaxSearchDeep:
                    MaxSearchDeep = 1 + MaxSearchDeep
        if len(Queue) > MaxFrontier:
            QueueSize = len(Queue)
            MaxFrontier = QueueSize

    print("There's not answer")
    return im_paths

#/********************DFS*********************/
def dfs(start, end, pixels, im_paths):
    global MaxFrontier, GoalNode, MaxSearchDeep
    
    color_paths = [128, 128, 128]
    visited = set()
    stack = list([start])

    while stack:
        node = stack.pop()
        arr = ','.join(str(x) for x in node.coordinates)
        visited.add(arr)
        im_paths[node.x, node.y] = (color_paths[0], 
                                    color_paths[1],
                                    color_paths[2])

        if node.x == end.x and node.y == end.y:
            GoalNode = node
            return im_paths

        possiblePaths = getAdjacent(pixels, node)
        for path in possiblePaths:
            path_map = ''
            for x in path.coordinates:
                path_map +=','.join(str(x))

            if path_map not in visited:
                stack.append(path)
                visited.add(path_map)
                if path.depth > MaxSearchDeep:
                    MaxSearchDeep = 1 + MaxSearchDeep
        if len(stack) > MaxFrontier:
            MaxFrontier = len(stack)

    print("There's not answer")
    return im_paths

#/**************************AST*************************/
def ast(start, end, pixels, im_paths):
    global MaxFrontier, GoalNode, MaxSearchDeep

    color_paths = [128, 128, 128]

    #Transform the initial state
    arr = ','.join(str(x) for x in start.coordinates)

    #Calculate Euclidean distance
    key = getDistance(pixels, start, end)
    visited = set()
    start.key = key
    Queue = []
    Queue.append(start)
    visited.add(arr)

    while Queue:
        Queue.sort(key=lambda o: o.key)
        node = Queue.pop(0)
        im_paths[node.x, node.y] = (color_paths[0], 
                                    color_paths[1],
                                    color_paths[2])

        if node.x == end.x and node.y == end.y:
            GoalNode = node
            return im_paths
        possiblePaths = getAdjacent(pixels, node)
        path_map = ''
        for path in possiblePaths:
            for x in path.coordinates:
                path_map +=','.join(str(x))
            if path_map not in visited:
                key = getDistance(pixels, path, end)
                path.key = key + path.depth
                Queue.append(path)
                visited.add(path_map)
                if path.depth > MaxSearchDeep:
                    MaxSearchDeep = 1 + MaxSearchDeep

    print("There's not answer")
    return im_paths

#/************************EUCLIDEAN DISTANCE*******************/
def getDistance(pixels, start, end):
    return 0.1 + (end.x - start.x)**2 + (end.y - start.y)**2

#/**************************MAIN*************************/
def main():
    global Width, Height, MaxFrontier, GoalNode, MaxSearchDeep, NodesExpanded

    # Open the maze image and make greyscale, and get its dimensions
    num = str(
        int(input('Enter the number of the maze that you want to solve: ')))
    maze = Image.open('./assets/maze{}.png'.format(num))
    im = maze.convert('L')
    Width, Height = im.size

    #Binarize the image
    binary = im.point(lambda p: p > 128 and 1)
    #Get the array of 0, 1

    im_bin = numpy.array(binary)
    x_i = int(input('Please, enter the start in axis X: '))
    y_i = int(input('Please, enter the start in axis Y: '))
    start = [y_i, x_i]

    pixel_start = Pixel(y_i, x_i, None, None, 0, 0, 0)

    x_f = int(input('Please, enter the end in axis X: '))
    y_f = int(input('Please, enter the end in axis Y: '))
    end = [y_f, x_f]
    pixel_end = Pixel(y_f, x_f, None, None, 0, 0, 0)
    GoalState = pixel_end.coordinates

    im_rgb = maze.convert('RGB')

    for i in range(0, 3):
        # Clear variables

        GoalNode = None
        NodesExpanded = 0
        MaxSearchDeep = 0
        MaxFrontier = 0
        im_rgb = maze.convert('RGB')
        im_paths = im_rgb.load()

        #Start timer to know how long each method takes 
        start = timeit.default_timer()
        if i == 0:
            method = 'BFS'
            im_solved = bfs(pixel_start, pixel_end, im_bin, im_paths)
        elif i == 1:
            method = 'DFS'
            im_solved = dfs(pixel_start, pixel_end, im_bin, im_paths)
        elif i == 2:
            method = 'AST'
            im_solved = ast(pixel_start, pixel_end, im_bin, im_paths)

        stop = timeit.default_timer()
        time = stop - start


        color_solution = [0, 198, 198]

        if GoalNode:
            deep = GoalNode.depth
            moves = []
            while pixel_start.coordinates != GoalNode.coordinates:
                im_solved[GoalNode.x, GoalNode.y] = (color_solution[0], 
                                                color_solution[1],
                                                color_solution[2])
                moves.insert(0, GoalNode.move)
                GoalNode = GoalNode.parent

            im_solved[pixel_start.x, pixel_start.y] = (color_solution[0], 
                                                color_solution[1],
                                                color_solution[2])
            im_rgb.save('./solvedMaze{}.png'.format(method))

            #print('Path: ',moves)
            print('Algorithm: {}'.format(method))
            print('Final Cost: ',len(moves))
            print('Nodes Expanded: ',str(NodesExpanded))
            print('Search depth: ',str(deep))
            print('MaxSearchDeep: ',str(MaxSearchDeep))
            print('Running Time: ',format(time, '.8f'))
            print('\n')

        else:
            im_rgb.save('./notSolvedMaze{}.png'.format(method))


if __name__ == '__main__':
    main()

#2
#X: 31, Y: 445
#X: 757, Y: 26

#3
#X:3 Y:0
#X; 7 Y:9

#4
#X: 221, Y:0
#X: 259, Y:584

#5
#X: 1, Y: 455
#X: 550, Y: 97

#6
#X: 230, Y:1
#X: 281, Y:507
