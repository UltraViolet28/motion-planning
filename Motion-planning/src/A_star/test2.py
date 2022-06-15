import cv2 
import numpy as np
import time
import math
import priority_dict

x_max = 380
y_max = 900


# cost function / Euclidean distance
def dist(a,b):
    return np.sqrt( (a[0] - b[0])**2 + (a[1] - b[1])**2 )

# heuristic function
def write_path(image, x, y):
    image = cv2.circle(image, (x,y), radius=0, color=(0, 0, 255))
    return image


# neighbouring nodes
def successors(node,map):
    neighbour = {}

    for i in range(-1,2):
        for j in range(-1,2):
            #print(1)

            y = node[0] + i
            x = node[1] + j

            if not((i,j) == (0,0)) and (y in range(map.shape[0]) and x in range(map.shape[1])):
                
                if map[y,x] == 0:
                    #map[y,x] = 255
                    neighbour[(y,x)] = dist(node,(y,x))

    
    return neighbour

# To Get the path 
def get_path(origin, goal, predecessor):
    key = goal
    path = [goal]
    
    while (key != origin):
        key = predecessor[key]
        path.insert(0, key)
        
    return path


# The A* algorithm
def a_star_search(start,goal,image):

    open_nodes = priority_dict.priority_dict({})
    closed = []
    predecessor = {}
    open_nodes[start] = 0.0

    while(open_nodes):
        u, u_cost = open_nodes.pop_smallest()

        if(u == goal):
            print("done")
            return get_path(predecessor, goal, start)

        for v_list in successors(u,image):
            
            v = v_list
            
            if v in closed:
                continue

            uv_cost = dist(v, u)
            h_v = dist(v, goal)

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


img = np.zeros((512,512,3), np.uint8)
#img = cv2.circle(img,(200,200), 20, (255,255,255), -1)
img = cv2.circle(img,(300,200), 70, (255,255,255), -1)
img = cv2.circle(img,(200,300), 75, (255,255,255), -1)
#img = cv2.rectangle(img,(184,100),(410,228),(255,255,255),-1)
gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

start = (10,40)
goal = (10,50)

path = a_star_search(start,goal,gray_img)

for i in path:
    write_path(img, i[0], i[1])


cv2.imshow('gray_img',gray_img)
cv2.imshow('img',img)
cv2.waitKey(0)
cv2.destroyAllWindows()





