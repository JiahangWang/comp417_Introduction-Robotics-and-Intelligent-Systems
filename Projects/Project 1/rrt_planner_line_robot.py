import time
import random
import drawSample
import math
import sys
import imageToRects
import utils


def redraw(canvas):
    canvas.clear()
    canvas.markit(tx, ty, r=SMALLSTEP)
    drawGraph(G, canvas)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")


def drawGraph(G, canvas):
    global lines, nodes, edges
    if not visualize: return
    for i in G[edges]:
        canvas.polyline([lines[i[0]], lines[i[1]]])


def genPoint():
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
            canvas.polyline([lines[e[0]], lines[e[1]]], style=3)
            return e[0]


def genvertex():
    lines.append(genPoint())
    return len(lines) - 1


def lineToVertex(p):
    lines.append(p)
    return len(lines) - 1


def pickvertex():
    return random.choice(range(len(lines)))


def LinePointDistance(l, p):
    x_start, y_start, direction = l
    x_p, y_p = p

    x_end = x_start + robot_length * math.cos(direction)
    y_end = y_start + robot_length * math.sin(direction)

    vx, vy = x_end - x_start, y_end - y_start
    wx, wy = x_p - x_start, y_p - y_start

    c = (wx * vx + wy * vy) / (vx * vx + vy * vy)

    if c < 0:
        distance = math.sqrt(wx * wx + wy * wy)
    elif c > 1:
        distance = math.sqrt((x_p - x_end) ** 2 + (y_p - y_end) ** 2)
    else:
        px = x_start + c * vx
        py = y_start + c * vy
        distance = math.sqrt((x_p - px) ** 2 + (y_p - py) ** 2)

    return distance


def distance_point_to_line(px, py, x1, y1, x2, y2):
    line_length_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2
    if line_length_sq == 0:
        return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)
    t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / line_length_sq
    t = max(0, min(1, t))
    proj_x = x1 + t * (x2 - x1)
    proj_y = y1 + t * (y2 - y1)
    return math.sqrt((proj_x - px) ** 2 + (proj_y - py) ** 2)


def closestLineToPoint(G, p):
    min_distance = float('inf')
    closest_line_index = None
    px, py = p

    for line_index in G[0]:  # G[0] contains the indices of lines
        line = lines[line_index]  # Get the line from the lines list using the index
        x, y, theta = line  # Extract the start point and direction of the line
        x_end, y_end = x + robot_length * math.cos(theta), y + robot_length * math.sin(
            theta)  # Calculate the end point of the line

        distance = distance_point_to_line(px, py, x, y, x_end,
                                          y_end)  # Calculate the distance between the given point and the line

        if distance < min_distance:
            min_distance = distance
            closest_line_index = line_index  # Store the index of the closest line

    return closest_line_index  # Return the index of the closest line


def on_segment(x, y, x1, y1, x2, y2):
    """Check if point (x, y) is on the line segment between (x1, y1) and (x2, y2)"""
    if min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2):
        return True
    return False


def line_intersects_rect(start_x, start_y, end_x, end_y, rect):
    left, top, right, bottom = rect

    # Define the lines of the rectangle
    rect_lines = [
        [(left, top), (right, top)],  # Top line
        [(right, top), (right, bottom)],  # Right line
        [(right, bottom), (left, bottom)],  # Bottom line
        [(left, bottom), (left, top)]  # Left line
    ]

    for line in rect_lines:
        x1, y1 = line[0]
        x2, y2 = line[1]

        # Check whether the given line segment intersects with any line segment of the rectangle
        denom = ((y2 - y1) * (end_x - start_x)) - ((x2 - x1) * (end_y - start_y))
        if denom == 0:  # parallel lines
            if on_segment(start_x, start_y, x1, y1, x2, y2) and on_segment(end_x, end_y, x1, y1, x2, y2):
                continue  # Skip if the line segment is on one side of the rectangle
            else:
                return False

        ua = (((x2 - x1) * (start_y - y1)) - ((y2 - y1) * (start_x - x1))) / denom
        ub = (((end_x - start_x) * (start_y - y1)) - ((end_y - start_y) * (start_x - x1))) / denom

        if 0 < ua < 1 and 0 < ub < 1:
            return True

    return False


def lineHitsRect(l1, l2, o):
    x1, y1, d1 = l1
    x2, y2, d2 = l2

    end_x1 = x1 + robot_length * math.cos(d1)
    end_y1 = y1 + robot_length * math.sin(d1)

    end_x2 = x2 + robot_length * math.cos(d2)
    end_y2 = y2 + robot_length * math.sin(d2)

    if line_intersects_rect(x1, y1, end_x1, end_y1, o):
        return True

    theta = d2 - d1
    rot_end_x1 = x1 + robot_length * math.cos(d1 + theta)
    rot_end_y1 = y1 + robot_length * math.sin(d1 + theta)

    if line_intersects_rect(x1, y1, rot_end_x1, rot_end_y1, o):
        return True

    if line_intersects_rect(x1, y1, x2, y2, o):
        return True

    return False


def lineIntersectsLine(x1, y1, x2, y2, x3, y3, x4, y4):
    def ccw(Ax, Ay, Bx, By, Cx, Cy):
        return (Cy - Ay) * (Bx - Ax) > (By - Ay) * (Cx - Ax)

    # 检查线段AB和线段CD是否相交
    return ccw(x1, y1, x3, y3, x4, y4) != ccw(x2, y2, x3, y3, x4, y4) and ccw(x1, y1, x2, y2, x3, y3) != ccw(x1, y1, x2,
                                                                                                             y2, x4, y4)


def inRect(l, rect, dilation):
    x_start, y_start, theta = l
    left, top, right, bottom = rect

    # 计算线段的终点坐标
    x_end = x_start + robot_length * math.cos(theta)
    y_end = y_start + robot_length * math.sin(theta)

    # 扩张矩形
    left_dilated = left - dilation
    top_dilated = top - dilation
    right_dilated = right + dilation
    bottom_dilated = bottom + dilation

    # 检查线段的端点是否在扩张后的矩形内
    if (left_dilated <= x_start <= right_dilated and top_dilated <= y_start <= bottom_dilated) or \
            (left_dilated <= x_end <= right_dilated and top_dilated <= y_end <= bottom_dilated):
        return True

    # 检查线段是否与扩张后的矩形的四条边相交
    if lineIntersectsLine(x_start, y_start, x_end, y_end, left_dilated, top_dilated, right_dilated, top_dilated) or \
            lineIntersectsLine(x_start, y_start, x_end, y_end, right_dilated, top_dilated, right_dilated,
                               bottom_dilated) or \
            lineIntersectsLine(x_start, y_start, x_end, y_end, right_dilated, bottom_dilated, left_dilated,
                               bottom_dilated) or \
            lineIntersectsLine(x_start, y_start, x_end, y_end, left_dilated, bottom_dilated, left_dilated, top_dilated):
        return True

    return False


def addNewLine(line_index, p, stepsize):

    x_start, y_start, theta_old = lines[line_index]

    px, py = p
    delta_x, delta_y = px - x_start, py - y_start
    theta_new = (math.atan2(delta_y, delta_x) + 2 * math.pi) % (2 * math.pi)

    if theta_new > SMALLTHETA:
        theta_new = SMALLTHETA

    x_start_new = x_start + stepsize * math.cos(theta_new)
    y_start_new = y_start + stepsize * math.sin(theta_new)

    return [x_start_new, y_start_new, theta_new]


def rrt_search(G, tx, ty, canvas):
    iters = 0
    # TODO
    # Fill this function as needed to work ...
    global sigmax_for_randgen, sigmay_for_randgen
    n = 0
    nsteps = 0
    while 1:
        iters += 1
        p = genPoint()
        cp = closestLineToPoint(G, p)
        v = addNewLine(cp, p, SMALLSTEP)

        if visualize:
            # if nsteps%500 == 0: redraw()  # erase generated points now and then or it gets too cluttered
            n = n + 1
            if n > 10:
                canvas.events()
                n = 0

        flag = False
        for o in obstacles:
            tmp = lines[cp]
            if lineHitsRect(lines[cp], v, o) or inRect(v, o, 1):
                # Skip the current iteration as the line or point is obstructed by an obstacle
                # print("障碍")
                flag = True

        if flag:
            continue

        # print("通过----------------")
        k = lineToVertex(v)  # is the new vertex ID
        G[nodes].append(k)
        G[edges].append((cp, k))

        if visualize:
            canvas.polyline([lines[cp], lines[k]])

        if LinePointDistance(v, [tx, ty]) < SMALLSTEP:
            print("Target achieved.", nsteps, "nodes in entire tree")
            print(f"iterations = {iters}")
            if visualize:
                t = lineToVertex([tx, ty])  # is the new vertex ID
                G[edges].append((k, t))
                if visualize:
                    canvas.polyline([p, lines[t]], 1)
                # while 1:
                #     # backtrace and show the solution ...
                #     canvas.events()
                nsteps = 0
                totaldist = 0
                while 1:
                    oldp = lines[k][0:2]  # remember point to compute distance
                    k = returnParent(k, canvas)  # follow links back to root.
                    canvas.events()
                    if k <= 1: break  # have we arrived?
                    nsteps = nsteps + 1  # count steps
                    totaldist = totaldist + LinePointDistance(lines[k], oldp)  # sum lengths
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
        redraw(canvas)
        G[edges].append((0, 1))
        G[nodes].append(1)
        if visualize: canvas.markit(tx, ty, r=SMALLSTEP)

        drawGraph(G, canvas)
        rrt_search(G, tx, ty, canvas)

    if visualize:
        canvas.mainloop()


if __name__ == '__main__':
    # display = drawSample.SelectRect(imfile=im2Small,keepcontrol=0,quitLabel="")
    args = utils.get_args()
    visualize = utils.get_args()
    drawInterval = 100  # 10 is good for normal real-time drawing

    prompt_before_next = 1  # ask before re-running sonce solved
    SMALLSTEP = args.step_size  # what our "local planner" can handle.

    SMALLTHETA = 5  # degrees

    map_size, obstacles = imageToRects.imageToRects(args.world)
    # Note the obstacles are the two corner points of a rectangle
    # Each obstacle is (x1,y1), (x2,y2), making for 4 points
    XMAX = map_size[0]
    YMAX = map_size[1]

    G = [[0], []]  # nodes, edges
    # lines = [((args.start_pos_x, args.start_pos_y), args.start_pos_theta),
    #             ((args.start_pos_x, args.start_pos_y + 10), args.start_pos_theta)]

    lines = [(args.start_pos_x, args.start_pos_y, args.start_pos_theta),
             (args.start_pos_x, args.start_pos_y + 10, args.start_pos_theta)]

    robot_length = args.robot_length

    # goal/target
    tx = args.target_pos_x
    ty = args.target_pos_y

    # start
    sigmax_for_randgen = XMAX / 2.0
    sigmay_for_randgen = YMAX / 2.0
    nodes = 0
    edges = 1
    main()
