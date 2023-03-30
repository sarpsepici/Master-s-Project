import numpy as np
import random
import cv2
"""
def check_collision(bitmap, p1, p2):
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    m = (y2 - y1) / (x2 - x1) if x2 != x1 else float('inf')
    if m == float('inf'):
        for y in range(min(y1, y2), max(y1, y2) + 1):
            if bitmap[x1][y] == 0:
                return True
    elif m == 0:
        for x in range(min(x1, x2), max(x1, x2) + 1):
            if bitmap[x][y1] == 0:
                return True
    else:
        for x in range(min(x1, x2), max(x1, x2) + 1):
            y = m * (x - x1) + y1
            if bitmap[x][int(y)] == 0:
                return True
    return False

def check_collision(bitmap, p1, p2, safety_distance):
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    m = (y2 - y1) / (x2 - x1) if x2 != x1 else float('inf')
    if m == float('inf'):  # vertical line
        for y in range(min(y1, y2), max(y1, y2) + 1):
            if bitmap[x1][y] == 0:
                return True
            if any([bitmap[x1+safety_distance][y]==0, bitmap[x1-safety_distance][y]==0]):
                return True
    elif m == 0:  # horizontal line
        for x in range(min(x1, x2), max(x1, x2) + 1):
            if bitmap[x][y1] == 0:
                return True
            if any([bitmap[x][y1+safety_distance]==0, bitmap[x][y1-safety_distance]==0]):
                return True
    else:  # non-vertical and non-horizontal line
        for x in range(min(x1, x2), max(x1, x2) + 1):
            y = m * (x - x1) + y1
            if bitmap[x][int(y)] == 0:
                return True
            if any([bitmap[x+safety_distance][int(y+safety_distance)]==0, bitmap[x-safety_distance][int(y-safety_distance)]==0]):
                return True
    return False

def check_collision(bitmap, p1, p2, safety_distance):
    x1, y1 = p1
    x2, y2 = p2
    dx, dy = x2 - x1, y2 - y1
    length = np.sqrt(dx ** 2 + dy ** 2)
    if length == 0:  # zero length line
        return False
    dx, dy = dx / length, dy / length  # unit vector in direction of line
    px, py = x1, y1  # current position on line
    while True:
        # check for collision with current position and safety distance
        if bitmap[int(px)][int(py)] == 0:
            return True
        for dist in range(10, safety_distance*10 + 10):
            if bitmap[int(px + dist/10 * dx)][int(py + dist/10 * dy)] == 0:
                return True
            if bitmap[int(px - dist/10 * dx)][int(py - dist/10 * dy)] == 0:
                return True
        # move to next position on line
        px += dx
        py += dy
        if (x2 - px) * dx <= 0 and (y2 - py) * dy <= 0:  # reached end of line
            break
    return False

def check_collision(bitmap, p1, p2, safety_distance=10):
    x1, y1 = p1
    x2, y2 = p2
    dx, dy = x2 - x1, y2 - y1
    length = np.sqrt(dx ** 2 + dy ** 2)
    dx, dy = dx / length, dy / length  # unit vector in direction of line
    px, py = x1, y1  # current position on line
    while True:
        # check for collision with current position and safety distance
        for dist in range(safety_distance*10, 0, -1):
            if bitmap[int(px + dist/10 * dx)][int(py + dist/10 * dy)] == 0:
                return False
        # move to next position on line
        px += dx
        py += dy
        if (x2 - px) * dx <= 0 and (y2 - py) * dy <= 0:  # reached end of line
            break
    return True

def check_collision(bitmap, p1, p2, safety_dist=10):
    radius = 10
    x1, y1 = p1
    x2, y2 = p2
    dx, dy = x2 - x1, y2 - y1
    length = np.sqrt(dx ** 2 + dy ** 2)
    if length == 0:  # zero length line
        return False
    dx, dy = dx / length, dy / length  # unit vector in direction of line
    for dist in range(safety_dist*10 + radius, int(length*10) - radius, radius):
        # check for collision with obstacles around the line
        for angle in range(0, 360, 5):
            x = int(x1 + (dist/10) * np.cos(np.deg2rad(angle)))
            y = int(y1 + (dist/10) * np.sin(np.deg2rad(angle)))
            if x >= 0 and x < bitmap.shape[1] and y >= 0 and y < bitmap.shape[0]:
                if bitmap[y, x] == 0:
                    return False
    return True
"""

def check_collision(bitmap, start_pixel, end_pixel, safety_dist):
    # Bresenham's line algorithm to generate the line
    x0, y0 = start_pixel
    x1, y1 = end_pixel
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    line_pixels = []
    while True:
        line_pixels.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    # Check collision for each pixel on the line
    for pixel in line_pixels:
        x, y = pixel
        for i in range(max(0, x - safety_dist), min(300, x + safety_dist + 1)):
            for j in range(max(0, y - safety_dist), min(300, y + safety_dist + 1)):
                if bitmap[i][j] == 0 and np.sqrt((x-i)**2 + (y-j)**2) <= safety_dist:
                    return False
    return True

"""
def generate_lines(bitmap, max_distance, start, goal):

    # 300 random points in bitmap.
    points = [start, goal]
    while len(points) < 750:
        x = random.randint(10, bitmap.shape[1]-11)
        y = random.randint(10, bitmap.shape[0]-11)
        if bitmap[y, x] == 1:
            points.append((x, y))

    lines_dict = {point: [] for point in points}

    for i in range(len(points)):
        for j in range(i+1, len(points)):

            p1, p2 = points[i], points[j]

            if np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2) <= max_distance:

                if check_collision(bitmap, p1, p2, 5):
                    break
                else:
                    lines_dict[tuple(p1)].append(tuple(p1))
                    lines_dict[tuple(p1)].append(tuple(p2))
                    lines_dict[tuple(p2)].append(tuple(p1))
                    lines_dict[tuple(p2)].append(tuple(p2))

    return lines_dict
"""

def generate_lines(bitmap, max_distance, start, goal):

    # 300 random points in bitmap.
    points = [start, goal]
    while len(points) < 5000:
        x = random.randint(50, bitmap.shape[1]-51)
        y = random.randint(50, bitmap.shape[0]-51)
        if bitmap[y, x] == 1:
            points.append((x, y))

    lines_dict = {point: [] for point in points}

    for i in range(len(points)):

        # find 10 closest points to form a line with without collision

        distances = [(np.sqrt((points[i][0]-p[0])**2 + (points[i][1]-p[1])**2), p) for p in points if p != points[i]]
        distances.sort()
        closest_points = [p[1] for p in distances[:10]]

        for j in range(len(closest_points)):
            p1, p2 = points[i], closest_points[j]

            if np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2) <= max_distance:

                if not check_collision(bitmap, p1, p2, 30):
                    continue
                else:
                    if len(lines_dict[tuple(p1)]) < 20:
                        lines_dict[tuple(p1)].append(tuple(p1))
                        lines_dict[tuple(p1)].append(tuple(p2))
                    if len(lines_dict[tuple(p2)]) < 20:
                        lines_dict[tuple(p2)].append(tuple(p1))
                        lines_dict[tuple(p2)].append(tuple(p2))

    return lines_dict

class Astar():

    def heuristic(self, u, v):
        
        dist = np.sqrt((u[0] - v[0])**2 + (u[1] - v[1])**2)
        return dist

    def distance_gen(self, u, v):
    
        dist = np.sqrt((u[0] - v[0])**2 + (u[1] - v[1])**2)
        return dist
        
    def path(self, start, goal, lines_dict):
        
        Q = {start: 0}
        cost2reach_d = {start: 0}
        parents = {start: start}
        path0 = []

        while len(Q)>0:

            Q0 = (list(Q.keys()))[0]

            if Q0 == goal:
                path0 = [goal]
                while parents[path0[0]] != start:
                    path0.insert(0, parents[path0[0]])
                if start != goal:
                    path0.insert(0, start)
                return path0

            succ = lines_dict[Q0]

            for i in succ:

                wgt = self.distance_gen(Q0, i)
                cost2reach = cost2reach_d[Q0] + wgt
                heur = self.heuristic(i, goal)

                if i in list(cost2reach_d.keys()):
                    if cost2reach_d[i] > cost2reach:
                        cost2reach_d[i] = cost2reach
                        if i in list(Q.keys()):
                            Q[i] = heur + cost2reach
                        parents[i] = Q0

                if i not in list(parents.keys()):
                    parents[i] = Q0
                    cost2reach_d[i] = cost2reach
                    Q[i] = heur + cost2reach

            Q.pop(Q0)
            Q_temp = sorted(Q.items(), key=lambda x:x[1])
            Q = dict(Q_temp)

        return []


def generate_bitmap(size, num_circles):
    bitmap = np.ones((size, size))
    
    radius = 40
    x = 150
    y = 150
    for j in range(size):
        for k in range(size):
            if ((j-y)**2 + (k-x)**2) < radius**2:
                bitmap[j, k] = 0
    return bitmap

start = tuple([30,30])
goal = tuple([200,200])

bitmap = np.ones((300,300))
bitmap = generate_bitmap(300, 1)

lines_dict_test = generate_lines(bitmap, 100, start, goal)
astar_test = Astar()

path_test = astar_test.path(start, goal, lines_dict=lines_dict_test)

if path_test is not None:
    path_img = bitmap.copy()
    for i in range(len(path_test)-1):
        cv2.line(path_img, path_test[i], path_test[i+1], (0, 0, 255), 2)
    cv2.imshow('Path', path_img)
    cv2.waitKey(0)

