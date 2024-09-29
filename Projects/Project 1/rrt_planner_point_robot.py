'''
Hello and welcome to the first assignment :)
Please try to enjoy implementing algorithms you learned from the course; in this regard, we also have tried
to provide a good starting point for you to develop your own code. In this script, you may find different functions
that should be defined by you to perform specific tasks. A good suggestion would be to begin with the primary
function for the RRT algo named as "rrt_search". Following this function line by line you may realize how to define
each function to make it work!
Note, you may use the following commands to run this script with the required arguments:
python3 rrt_planner_point_robot.py --arg1_name your_input1 --arg2_name your_input2 e.g.
python3 rrt_planner_point_robot.py --world="shot.png"
To see the list of arguments, take a look at utils.py
Also note:
G is a list containing two groups: the first group includes the nodes IDs (0,1,2,...), while the second holds all pairs of nodes creating an edge
Vertices is a list of nodes locations in the map; it corresponds to the nodes' IDs mentioned earlier
GOOD LUCK!
'''

import random
import drawSample
import sys
import imageToRects
import utils
import math


def redraw(canvas):
    canvas.clear()
    canvas.markit(tx, ty, r=SMALLSTEP)
    drawGraph(G, canvas)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")


def drawGraph(G, canvas):
    global vertices, nodes, edges
    if not visualize: return
    for i in G[edges]:
        # e.g. vertices: [[10, 270], [10, 280]]
        canvas.polyline([vertices[i[0]], vertices[i[1]]])


# Use this function to generate points randomly for the RRT algo
def genPoint():
    # if args.rrt_sampling_policy == "uniform":
    #     # Uniform distribution
    #     x = random.random()*XMAX
    #     y = random.random()*YMAX
    # elif args.rrt_sampling_policy == "gaussian":
    #     # Gaussian with mean at the goal
    #     x = random.gauss(tx, sigmax_for_randgen)
    #     y = random.gauss(ty, sigmay_for_randgen)
    # else:
    #     print ("Not yet implemented")
    #     quit(1)

    bad = 1
    while bad:
        bad = 0
        if args.rrt_sampling_policy == "uniform":
            # Uniform distribution
            x = random.random() * XMAX
            y = random.random() * YMAX
        elif args.rrt_sampling_policy == "gaussian":
            # Gaussian with mean at the goal
            x = random.gauss(tx, sigmax_for_randgen)
            y = random.gauss(ty, sigmay_for_randgen)
        else:
            print("Not yet implemented")
            quit(1)
        # range check for gaussian
        if x < 0: bad = 1
        if y < 0: bad = 1
        if x > XMAX: bad = 1
        if y > YMAX: bad = 1
    return [x, y]


def returnParent(k, canvas):
    """ Return parent note for input node k. """
    for e in G[edges]:
        if e[1] == k:
            canvas.polyline([vertices[e[0]], vertices[e[1]]], style=3)
            return e[0]


def genvertex():
    vertices.append(genPoint())
    return len(vertices) - 1


def pointToVertex(p):
    vertices.append(p)
    return len(vertices) - 1


def pickvertex():
    return random.choice(range(len(vertices)))


def lineFromPoints(p1, p2):
    # TODO
    return [None]


def pointPointDistance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    # Calculate the Euclidean distance between the two points
    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance


def closestPointToPoint(G, p):
    min_distance = float('inf')
    closest_point_index = None

    for vertex_index in G[0]:  # Assuming G[0] contains the indices of vertices
        vertex = vertices[vertex_index]  # Get the coordinates of the vertex from the vertices list
        distance = ((vertex[0] - p[0]) ** 2 + (vertex[1] - p[1]) ** 2) ** 0.5  # Calculate Euclidean distance

        if distance < min_distance:
            min_distance = distance
            closest_point_index = vertex_index  # Store the index of the closest vertex

    return closest_point_index  # Return the index of the closest vertex


def lineHitsRect(p1, p2, r):
    # r is a rectangle represented as [x1, y1, x2, y2]
    x1, y1, x2, y2 = r  # Extract the coordinates from the rectangle

    # Define the four lines representing the edges of the rectangle
    rect_lines = [
        ((x1, y1), (x1, y2)),  # Left edge
        ((x1, y1), (x2, y1)),  # Top edge
        ((x2, y1), (x2, y2)),  # Right edge
        ((x1, y2), (x2, y2))  # Bottom edge
    ]

    for line in rect_lines:
        if lineIntersectsLine(p1, p2, line[0], line[1]):
            return True  # The line segment intersects with one of the rectangle's edges

    return False  # The line segment does not intersect with any of the rectangle's edges


def lineIntersectsLine(l1p1, l1p2, l2p1, l2p2):
    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    return ccw(l1p1, l2p1, l2p2) != ccw(l1p2, l2p1, l2p2) and ccw(l1p1, l1p2, l2p1) != ccw(l1p1, l1p2, l2p2)


def inRect(p, rect, dilation):
    # rect is represented as [x1, y1, x2, y2]
    x1, y1, x2, y2 = rect

    # Apply dilation to the rectangle
    x1 -= dilation
    y1 -= dilation
    x2 += dilation
    y2 += dilation

    # Check if the point p is inside the dilated rectangle
    px, py = p
    return x1 <= px <= x2 and y1 <= py <= y2


def addNewPoint(p1_index, p2, stepsize):
    p1 = vertices[p1_index]  # Get the coordinates of p1 from the vertices list

    # Calculate the distance between p1 and p2
    distance = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    # If the distance between p1 and p2 is less than or equal to stepsize, return p2 directly
    if distance <= stepsize:
        return p2

    # Calculate the coordinates of the new point
    ratio = stepsize / distance  # Calculate the ratio
    x_new = p1[0] + ratio * (p2[0] - p1[0])  # x coordinate of the new point
    y_new = p1[1] + ratio * (p2[1] - p1[1])  # y coordinate of the new point

    return [x_new, y_new]  # Return the coordinates of the new point


def rrt_search(G, tx, ty, canvas, ):
    numiters = 0
    # Please carefully read the comments to get clues on where to start
    # TODO
    # Fill this function as needed to work ...
    global sigmax_for_randgen, sigmay_for_randgen
    n = 0
    nsteps = 0

    while 1:  # Main loop
        numiters += 1
        # This generates a point in form of [x,y] from either the normal dist or the Gaussian dist
        p = genPoint()

        # This function must be defined by you to find the closest point in the existing graph to the guiding point
        cp = closestPointToPoint(G, p)
        v = addNewPoint(cp, p, SMALLSTEP)

        if visualize:
            # if nsteps%500 == 0: redraw()  # erase generated points now and then or it gets too cluttered
            n = n + 1
            if n > 10:
                canvas.events()
                n = 0

        flag = False
        for o in obstacles:
            # The following function defined by you must handle the occlusion cases
            if lineHitsRect(vertices[cp], v, o) or inRect(v, o, 5):
                # Skip the current iteration as the line or point is obstructed by an obstacle
                # print("障碍")
                flag = True

        if flag:
            continue

        # print("通过")
        k = pointToVertex(v)  # is the new vertex ID
        G[nodes].append(k)
        G[edges].append((cp, k))
        if visualize:
            canvas.polyline([vertices[cp], vertices[k]])

        if pointPointDistance(v, [tx, ty]) < SMALLSTEP:
            print("Target achieved.", nsteps, "nodes in entire tree")
            print(f"iterations = {numiters}")
            if visualize:
                t = pointToVertex([tx, ty])  # is the new vertex ID
                G[edges].append((k, t))
                if visualize:
                    canvas.polyline([p, vertices[t]], 1)
                # while 1:
                #     # backtrace and show the solution ...
                #     canvas.events()
                nsteps = 0
                totaldist = 0

                while 1:
                    oldp = vertices[k]  # remember point to compute distance
                    k = returnParent(k, canvas)  # follow links back to root.
                    canvas.events()
                    if k <= 1: break  # have we arrived?
                    nsteps = nsteps + 1  # count steps
                    totaldist = totaldist + pointPointDistance(vertices[k], oldp)  # sum lengths
                print("Path length", totaldist, "using", nsteps, "nodes.")

                global prompt_before_next
                if prompt_before_next:
                    canvas.events()
                    print("More [c,q,g,Y]>")
                    d = sys.stdin.readline().strip().lstrip()
                    print("[" + d + "]")
                    if d == "c": canvas.delete()
                    if d == "q": return
                    if d == "g": prompt_before_next = 0
                break


def main():
    # seed
    random.seed(args.seed)
    if visualize:
        canvas = drawSample.SelectRect(xmin=0, ymin=0, xmax=XMAX, ymax=YMAX, nrects=0,
                                       keepcontrol=0)  # , rescale=800/1800.)
        for o in obstacles: canvas.showRect(o, outline='red', fill='blue')
    while 1:
        # graph G
        redraw(canvas)
        G[edges].append((0, 1))
        G[nodes].append(1)
        if visualize: canvas.markit(tx, ty, r=SMALLSTEP)
        drawGraph(G, canvas)
        rrt_search(G, tx, ty, canvas)

    if visualize:
        canvas.mainloop()


if __name__ == '__main__':
    args = utils.get_args()
    visualize = utils.get_args()
    drawInterval = 10  # 10 is good for normal real-time drawing

    prompt_before_next = 1  # ask before re-running sonce solved
    SMALLSTEP = args.step_size  # what our "local planner" can handle.
    map_size, obstacles = imageToRects.imageToRects(args.world)
    # Note the obstacles are the two corner points of a rectangle (left-top, right-bottom)
    # Each obstacle is (x1,y1), (x2,y2), making for 4 points

    XMAX = map_size[0]
    YMAX = map_size[1]
    # The boundaries of the world are (0,0) and (XMAX,YMAX)

    G = [[0], []]  # nodes, edges
    # vertices = [[args.start_pos_x, args.start_pos_y], [args.start_pos_x, args.start_pos_y + 10]]
    vertices = [(args.start_pos_x, args.start_pos_y), (args.start_pos_x, args.start_pos_y + 10)]

    # goal/target
    tx = args.target_pos_x
    ty = args.target_pos_y

    # start
    sigmax_for_randgen = XMAX / 2.0
    sigmay_for_randgen = YMAX / 2.0
    nodes = 0
    edges = 1

    main()
