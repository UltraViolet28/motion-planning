import cv2
import math
import priority_dict

x_max = 380
y_max = 900



def create_map():
    path = 'images/my_world_map.pgm'
    image = cv2.imread(path)
    img = image[1200:2100,1470:1850]
    img = cv2.threshold(img, 230, 255, cv2.THRESH_BINARY)[1]
    return img

def write_path(image, x, y):
    image = cv2.circle(image, (x,y), radius=0, color=(0, 0, 255))
    return image

def show_image(image):
    cv2.imshow('map', image) 
    cv2.waitKey(0) 
    cv2.destroyAllWindows() 
    
def eucledian_dist(point1, point2):
    x1 = point1 % x_max
    y1 = point1//x_max
    x2 = point2 % x_max
    y2 = point2//x_max

    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return dist

def get_values(image):
    x = int(input("  x: "))
    y = int(input("  y: "))

    while(x<0 or x>=x_max or y<0 or y>=y_max or image[y, x, 0] != 255):
        print("Invalid place. Please try again.")
        x = int(input("  x: "))
        y = int(input("  y: "))
    return x, y

def successors(image, point):
    x = point % x_max
    y = point//x_max

    succ = []
    if(image[y, x-1, 0] == 255):
        succ.append([x-1, y])
    if(image[y-1, x-1, 0] == 255):
        succ.append([x-1, y-1])
    if(image[y-1, x, 0] == 255):
        succ.append([x, y-1])
    if(image[y-1, x+1, 0] == 255):
        succ.append([x+1, y-1])
    if(image[y, x+1, 0] == 255):
        succ.append([x+1, y])
    if(image[y+1, x+1, 0] == 255):
        succ.append([x+1, y+1])
    if(image[y+1, x, 0] == 255):
        succ.append([x, y+1])
    if(image[y+1, x-1, 0] == 255):
        succ.append([x-1, y+1])
    
    return succ

def make_path(predecessor, goal, start):
    path = 'images/my_world_map.pgm'
    img= cv2.imread(path)
    img = img[1200:2100,1470:1850]
    key = goal
    x = key % x_max
    y = key//x_max
    write_path(image, x, y)
    
    while (key != start):
        x = key % x_max
        y = key//x_max
        write_path(img, x, y)
        key = predecessor[key]

    return img

def a_star(image, start, goal):
    open_nodes = priority_dict.priority_dict({})
    closed = []
    predecessor = {}
    open_nodes[start] = 0.0

    while(open_nodes):
        u, u_cost = open_nodes.pop_smallest()

        if(u == goal):
            print("done")
            return make_path(predecessor, goal, start)

        for v_list in successors(image, u):
            
            v = v_list[1] * x_max + v_list[0]
            
            if v in closed:
                continue

            uv_cost = eucledian_dist(v, u)
            h_v = eucledian_dist(v, goal)

            if v in open_nodes:
                v_cost = u_cost + uv_cost + h_v
                if v_cost < open_nodes[v]:
                    open_nodes[v] = v_cost
                    predecessor[v] = u
            else:
                open_nodes[v] = u_cost + uv_cost
                predecessor[v] = u

        closed.append(u)

    print("Goal not found")
    
image = create_map()

print("Start: ")
start_x, start_y = 340,230
start = start_y*x_max + start_x

print("Goal: ")
goal_x, goal_y = 330 , 400
goal = goal_y*x_max + goal_x

image = a_star(image, start, goal)

show_image(image)