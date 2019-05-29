#!/usr/bin/env python

##################
# Needed for ROS #
##################
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

#
# ###########
# # READ ME #
# ###########
# # in the maze:
# # 0 = nothing to consider
# # 1 = part of the planning zone
# # 2 = part of the safe_zone
# # 3 = cone
# # 4 = human
# # 5 = human safe zone


import matplotlib.pyplot as plt
import time
import numpy
import math

maze_x_size = 300
maze_y_size = 300
safe_zone = 10
planning_zone = 20
human_zone = 30
p_step_size = 3
end = (0, 180)
first_run = True

###################
# 0 = no turn     #
# 1 = left turn   #
# 2 = right turn  #
###################


last_turn = 0


maze3 = [[0] * maze_y_size for _ in range(maze_x_size)]
maze = maze3
start = (0.0, 0.0)
dot_size = 300/len(maze)
global maze_plot
global path_data
global safe_plot
global planning_plot
global travel_plot
global human_plot
global human_safe_plot
global reference_plot
planning_x = []
planning_y = []
safe_x = []
safe_y = []
maze_x = []
maze_y = []
travel_x = []
travel_y = []
human_x = []
human_y = []
human_safe_x = []
human_safe_y = []
reference_x = []
reference_y = []
step_size = 1
aim_distance = 5


# A position or Node in the map, used by astar to keep track
# of parents and so on
class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


# Takes the current position/node and returns the possible ways
# to travel, based on the direction of he last move
def get_possible_ways(current):
    # print(current.position)
    previous = current.parent
    possible_ways = []
    if previous is not None:
        x = current.position[0] - previous.position[0]
        y = current.position[1] - previous.position[1]
        if x == 0 and y == p_step_size:
            possible_ways = [(-p_step_size, p_step_size), (0, p_step_size), (p_step_size, p_step_size)]
        if x == p_step_size and y == p_step_size:
            possible_ways = [(0, p_step_size), (p_step_size, p_step_size), (p_step_size, 0)]
        if x == p_step_size and y == 0:
            possible_ways = [(p_step_size, p_step_size), (p_step_size, 0), (p_step_size, -p_step_size)]
        if x == p_step_size and y == -p_step_size:
            possible_ways = [(p_step_size, 0), (p_step_size, -p_step_size), (0, -p_step_size)]
        if x == 0 and y == -p_step_size:
            possible_ways = [(p_step_size, -p_step_size), (0, -p_step_size), (-p_step_size, -p_step_size)]
        if x == -p_step_size and y == -p_step_size:
            possible_ways = [(0, -p_step_size), (-p_step_size, -p_step_size), (-p_step_size, 0)]
        if x == -p_step_size and y == 0:
            possible_ways = [(-p_step_size, -p_step_size), (-p_step_size, 0), (-p_step_size, p_step_size)]
        if x == -p_step_size and y == p_step_size:
            possible_ways = [(-p_step_size, 0), (-p_step_size, p_step_size), (0, p_step_size)]
    else:
        # The following defines how the car is able to move when it starts, only forward
        possible_ways = [(-p_step_size, p_step_size), (0, p_step_size), (p_step_size, p_step_size)]

    return possible_ways


# Takes a maze (map) and a start and end position and generates
# a path between the positions
def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    #print(maze[0][0])
    if maze[0][0] > 1:
        print("You are in a nogo zone")
        return []
    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []
    nr = 0

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:
        if nr > 2000:
            return []
        nr = nr + 1
        print(nr)
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        # distance_left_square = distance(current_node.position, end_node.position)
        # print(distance_left_square)
        # if distance_left_square < pow(1, 2):
        distance_left_square = distance(current_node.position, end_node.position)
        if math.sqrt(distance_left_square) <= p_step_size:
            #print("reached the goal: " + str(all_goals))
        # if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        possible_ways = get_possible_ways(current_node)
        for new_position in possible_ways: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze)-1)/2 or node_position[0] < -(len(maze)-1)/2 or node_position[1] > (len(maze[len(maze)-1])-1) or node_position[1] < 0:
                print("Out of range")
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] == 1 or maze[node_position[0]][node_position[1]] == 2 or maze[node_position[0]][node_position[1]] == 3:
                print(maze[node_position[0]][node_position[1]])
                print("unwalkable")
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)
        # if (number_of_loops % 1000) == 1:
            # print(number_of_loops)
    return []


# Initiates the plots, only used once
def init_draw():
    plt.axes().set_aspect('equal')  # set the x and y axes to the same scale
    plt.axis([-(len(maze))/2, (len(maze))/2, 0, len(maze[0]) - 1])
    plt.grid()


# Get the end position by the user, from input
def get_end():
    raw_input = str(input('Enter end coordinates in "x,y" format: '))
    raw_input = raw_input.replace("(", "")
    raw_input = raw_input.replace(")", "")
    string_list = raw_input.split(",")
    return int(string_list[0]), int(string_list[1])


# As of now this method ony searches up, down, left and right
def closest_free_point(current_pos):
    new_start = current_pos
    distance_from_current = 1
    while maze[new_start[0]][new_start[1]] != 0:
        try:
            if maze[current_pos[0] + distance_from_current][current_pos[1]] == 0:
                new_start = (current_pos[0] + distance_from_current, current_pos[1])
            elif maze[current_pos[0] - distance_from_current][current_pos[1]] == 0:
                new_start = (current_pos[0] - distance_from_current, current_pos[1])
            distance_from_current = distance_from_current + 1
        except IndexError:
            #print("the search went out of map")
            continue
    return new_start


# Uses astar method to generate path, then generats two list of
# x and z/y values
def get_path(start, end):
    path_x = []
    path_y = []
    rounded_start = (int(round(start[0])), int(round(start[1])))
    if maze[rounded_start[0]][rounded_start[1]] == 1:
        rounded_start = closest_free_point(rounded_start)
        path = astar(maze, rounded_start, end)
        if path is not None:
            for i in range(len(path)):
                path_x.append(path[i][0])
                path_y.append(path[i][1])
            return path_x, path_y
    else:
        path = astar(maze, rounded_start, end)
        if path is not None:
            for i in range(len(path)):
                path_x.append(path[i][0])
                path_y.append(path[i][1])
            return path_x, path_y
    return [], [] # If the car is inside of the object or safety zone


# Used to process and and objects from the ROS message
def simulate_object_ros(message):
    one_object_list = message.split(";")
    global maze
    maze = [[0] * maze_y_size for _ in range(maze_x_size)]
    cone_x = []
    cone_y = []
    human_x_return = []
    human_y_return = []
    for obj in one_object_list:
        coords = obj.split(",")
        object_type = coords[len(coords) - 1].strip()
        if object_type == 'cone' or object_type == 'chair':
            for i in range(0, len(coords) - 1, 2):
                try:
                    maze[int(coords[i]) + int((len(maze))/2)][int(coords[i+1])] = 3
                except IndexError:
                    #  print("out of range")
                    continue
        if object_type == 'person':
            #print("found human")
            for i in range(0, len(coords) - 1, 2):
                #print(int(coords[i+1]))
                try:
                    maze[int(coords[i]) + int((len(maze))/2)][int(coords[i+1])] = 4
                except IndexError:
                    # print("out of range")
                    continue
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if maze[i][j] == 3:
                cone_x.append(i-int((len(maze))/2))
                cone_y.append(j)
            if maze[i][j] == 4:
                human_x_return.append(i-int((len(maze))/2))
                human_y_return.append(j)
    return cone_x, cone_y, human_x_return, human_y_return

# Generates a safe zone around objects (size is a global variable)
# Is used both when using ROS or testing alone
def generate_safe_zone(maze_x_value, maze_z_value):
    global maze
    global safe_x
    global safe_y
    safe_x = []
    safe_y = []
    number_of_points = len(maze_x_value)
    for point in range(number_of_points):  # loop through all dots
        for i in range(safe_zone*2):  # all x values around the object
            point_x = maze_x_value[point] + i - safe_zone
            for j in range(safe_zone*2):  # all z values around the object
                point_z = maze_z_value[point] + j - safe_zone
                try:
                    if (maze_x_value[point] - point_x)**2 + (maze_z_value[point] - point_z)**2 < safe_zone**2 and maze[point_x][point_z] == 0:
                        try:
                            maze[point_x][point_z] = 2
                            safe_x.append(point_x)
                            safe_y.append(point_z)
                        except IndexError:
                            # print("out of range, tied to add on map")
                            continue
                except IndexError:
                    # print("safe zone out of range")
                    continue


def generate_human_safe_zone(human_x_value, human_z_value):
    global maze
    global human_safe_x
    global human_safe_y
    human_safe_x = []
    human_safe_y = []
    number_of_points = len(human_x_value)
    #print("len = " + str(number_of_points))
    for point in range(number_of_points):  # loop through all dots
        for i in range(human_zone*2):  # all x values around the object
            point_x = human_x_value[point] + i - human_zone
            for j in range(human_zone*2):  # all z values around the object
                point_z = human_z_value[point] + j - human_zone
                try:
                    if (human_x_value[point] - point_x)**2 + (human_z_value[point] - point_z)**2 < human_zone**2 and \
                            maze[point_x][point_z] == 0:
                        try:
                            maze[point_x][point_z] = 5
                            human_safe_x.append(point_x)
                            human_safe_y.append(point_z)
                        except IndexError:
                            # print("out of range, tied to add on map")
                            continue
                except IndexError:
                    # print("safe zone out of range")
                    continue


def simulate_from_one_path(full_path):
    global start
    closest_path_coord = closest_coord(full_path, start)
    aim_index = full_path.index(closest_path_coord) + aim_distance
    if aim_index > len(full_path) - 1:
        aim_index = len(full_path) - 1
    aim_coord = full_path[aim_index]
    raw_vector_aim = tuple(numpy.subtract(aim_coord, start))
    raw_vector_length = numpy.sqrt(raw_vector_aim[0]**2 + raw_vector_aim[1]**2)
    unit_aim_vector = (float(format(step_size*raw_vector_aim[0]/raw_vector_length, '.2f')),
                       float(format(step_size*raw_vector_aim[1]/raw_vector_length, '.2f')))
    return unit_aim_vector


def distance(co1, co2):
    return pow(abs(co1[0] - co2[0]), 2) + pow(abs(co1[1] - co2[1]), 2)


def closest_coord(full_path, coord):
    closest = full_path[0]
    for c in full_path:
        if distance(c, coord) < distance(closest, coord):
            closest = c
    return closest


# Generates a zone in witch you are not allowed to plan your path through,
# you are however allowed to travel inside it once the plath is done.
# The program should then try to first get out of the zone and the continue on a normal path.
def generate_travel_zone(maze_x_value, maze_z_value):
    global maze
    global planning_x
    global planning_y
    planning_x = []
    planning_y = []
    number_of_points = len(maze_x_value)
    for point in range(number_of_points):  # loop through all dots
        for i in range((planning_zone + safe_zone) * 2):  # all x values around the object
            point_x = maze_x_value[point] + i - (planning_zone + safe_zone)
            for j in range((planning_zone + safe_zone) * 2):  # all z values around the object
                point_z = maze_z_value[point] + j - (planning_zone + safe_zone)
                try:
                    if (maze_x_value[point] - point_x)**2 + (maze_z_value[point] - point_z)**2 < \
                            (planning_zone + safe_zone)**2 and maze[point_x][point_z] == 0:
                        try:
                            maze[point_x][point_z] = 1
                            planning_x.append(point_x)
                            planning_y.append(point_z)
                        except IndexError:
                            # print("out of range, tied to add on map")
                            continue
                except IndexError:
                    # print("safe zone out of range")
                    continue

def check_turn(last_end, last_path_x, last_path_y):
    global reference_x
    global reference_y
    reference_x = []
    reference_y = []
    global start
    global last_turn
    if last_end[0] - start[0] == 0: #went straight
        #print("went straight")
        #print(int(round(last_end[1] - start[1])))
        for i in range (int(round(last_end[1] - start[1]))):
            reference_x.append(0)
            reference_y.append(i)
        #print(reference_x, reference_y)
    else:
        k = (last_end[1] - start[1]) / (last_end[0] - start[0])
        for i in range (last_end[1] - start[1]):
            reference_x.append(start[1] + i*k)
            reference_y.append(i)
    #print(last_path_x)
    #print(len(reference_x))
    '''''
    for i in range(len(last_path_x)):
        try:
            if last_path_x[i] - reference_x[last_path_y[i]] > 20:
                last_turn = 2
                #print("made a right turn")
                return
            if last_path_x[i] - reference_x[last_path_y[i]] < -20:
                last_path_y = 1
                #print("made a left turn")
                return
        except IndexError:
            #print("out of range, in check_turn")
            continue
    '''''



#Tells if the car should still follow the path
def keep_going(xs, ys):
    if xs == [] and ys == []:
        return False
    else:
        if pathValid(xs, ys):
            return True
        else:
            return False


#Check if the path is valid. Loop through the path and check if any pos is unavailable
def pathValid(xs, ys):
    global maze
    for i in range(0, len(xs)):
        if maze[int(xs[i])][int(ys[i])] > 1 and maze[int(xs[i])][int(ys[i])] < 3:
            return False

    return True

#rotate the path after the car moves
def shift_path(xs, ys, GPS, GPS_start):
    #print(xs)
    #print(type(GPS))
    GPSX, GPSY, GPST = GPS
    GPSXS, GPSYS, GPSTS = GPS_start

    x_gps_diff = float(GPSX-GPSXS)
    y_gps_diff = float(GPSY-GPSYS)

    x_rel = x_gps_diff*numpy.cos(-GPSTS) - y_gps_diff*numpy.sin(-GPSTS)
    y_rel = x_gps_diff*numpy.sin(-GPSTS) + y_gps_diff*numpy.cos(-GPSTS)
    t_r = GPST - GPSTS

    x_path = []
    y_path = []

    for i in range(len(xs)):
        x_rel_diff = xs[i]/10.0 - x_rel
        y_rel_diff = ys[i]/10.0 - y_rel

        x_path.append((x_rel_diff*numpy.cos(-t_r) - y_rel_diff*numpy.sin(-t_r))*10.0)
        y_path.append((x_rel_diff*numpy.sin(-t_r) + y_rel_diff*numpy.cos(-t_r))*10.0)
    
    global end
    if len(x_path):
        #print(x_path)
        end = (x_path[-1], y_path[-1])
    return x_path, y_path

def reduce_path(xs, ys, end):
    for i in range(0,len(xs)):
        if (xs[i], ys[i]) == end:
            return xs[:i], ys[:i]

    return xs, ys


#Calculate what to publish, a pos or None
def coords_to_pub(xs, ys):
    if xs == []:
        return None
    #calculate pos with least error
    steps_ahead = 15
    #print(xs)
    #print(ys)
    least_error = calc_error(xs[0], ys[0])
    least_error_index = 0
    for i in range(len(xs)):
        if least_error > calc_error(xs[i], ys[i]):
            least_error = calc_error(xs[i], ys[i])
            least_error_index = i
    if len(xs) == 0:
        return None
    elif len(xs)<=steps_ahead + least_error_index-1:
        return xs[-1], ys[-1]
    #print( xs[least_error_index+steps_ahead], ys[least_error_index+steps_ahead])
    return xs[least_error_index+steps_ahead], ys[least_error_index+steps_ahead]

def calc_error(x, y):
    return x**2+y**2
    
#return if the car should stop for a human
def stop(xs, ys, index, p):
    for i in range(p):
        try:
            if(maze[0][0]) > 1:
                return True
        except:
            return False
    return False

def calc_path(pp):
    a = time.time()
    global path_data
    global maze_plot
    global end
    global start
    global maze_x
    global maze_y
    global safe_x
    global safe_y
    global safe_plot
    global planning_plot
    global travel_plot
    global travel_x
    global travel_y
    global human_plot
    global human_safe_plot
    global human_x
    global human_y
    global human_safe_x
    global human_safe_y
    global reference_plot
    global first_run
    path_x = []
    path_y = []
    maze_x = []
    maze_y = []
    line_index = 0
#    end=(0,0)

    if first_run:
        first_run= False
        #print("initiated draw")
        plt.show()
        init_draw()
        safe_plot, = plt.plot(safe_x, safe_y, '.', color = "red")
        planning_plot, = plt.plot(planning_x, planning_y, '.', color = "yellow")
        human_safe_plot, = plt.plot(human_safe_x, human_safe_y, '.', color = "red")
        human_plot, = plt.plot(human_x, human_y, '.', color = "cyan", markersize=dot_size * 3)
        maze_plot, = plt.plot(maze_x, maze_y, '.', color = "green", markersize=dot_size*3)
        path_data, = plt.plot(path_x, path_y, 'g-', linewidth=1)
        travel_plot, = plt.plot(travel_x, travel_y, 'b-', linewidth=3)
       


    if distance(start, end) > step_size ** 2:
       # print('Distance to goal : ',distance(start, end)  step_size**2 )
        maze_x, maze_y, human_x, human_y = simulate_object_ros(pp.depth)
        generate_safe_zone(maze_x, maze_y)
        generate_human_safe_zone(human_x, human_y)
        generate_travel_zone(maze_x, maze_y)
        
        safe_plot.set_xdata(safe_x)
        safe_plot.set_ydata(safe_y)
        planning_plot.set_xdata(planning_x)
        planning_plot.set_ydata(planning_y)
        travel_plot.set_xdata(travel_x)
        travel_plot.set_ydata(travel_y)
        
        maze_plot.set_xdata(maze_x)
        maze_plot.set_ydata(maze_y)
        human_plot.set_xdata(human_x)
        human_plot.set_ydata(human_y)
        human_safe_plot.set_xdata(human_safe_x)
        human_safe_plot.set_ydata(human_safe_y)
        plt.draw()
        #print("drawn")
        plt.pause(1e-17)
        line_index = line_index + 1
        
        
####################################
        #print('before shit')
        path_x, path_y = shift_path(pp.xs, pp.ys, pp.GPS, pp.GPS_start)
        path_x = [int(i) for i in path_x]
        path_y = [int(i) for i in path_y]

        #print(path_y)
        #print('after shit')
        if not keep_going(path_x, path_y) or maze[0][0]>1:
            print('before path')
            pp.xs, pp.ys = get_path(start, end)
            #print(pp.xs)
            #print(pp.ys)
            print('after path')
            pp.GPS_start = pp.GPS
        

#####################################
        #print("updating plot")
        path_data.set_xdata(path_x)
        path_data.set_ydata(path_y)

    else:
        print('At goal :D')
        return 'done'        

    #print(path_x)
    return coords_to_pub(path_x, path_y)


class pp:
    def __init__(self):
        self.GPS = None
        self.depth = None
        self.xs = []
        self.ys = []
        self.GPS_start = (0.0,0.0,0.0)

    def GPS_callback(self, msg):
        # "Store" message received.
        self.GPS = msg.data
        #print(msg.data)

    def depth_callback(self, msg):
        # "Store" the message received.
        self.depth = msg.data

    def calculate(self):
        if self.depth is None:
            print('self.depth is None')
        if self.GPS is None:
            print('self.GPS is None')
        if self.depth is not None and self.GPS is not None:
            return calc_path(self)
        return None


# This is the method that decides if it should run on ROS
# message or the text-file
if __name__ == '__main__':
    pp = pp()
    rospy.init_node('pathplanner')
    pub = rospy.Publisher('aim_coords', String, queue_size=10)
    msg_to_publish = String()
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rospy.Subscriber('depth_output', String , pp.depth_callback)
        rospy.Subscriber('GPS_pos', Float32MultiArray, pp.GPS_callback)
        ret = pp.calculate()
        if not ret is None:
            #x,y = pp.calculate()
            msg_to_publish.data = str(ret)[1:len(str(ret))-1]
        else:
            msg_to_publish.data = 'stop'
        
        pub.publish(msg_to_publish)
        rate.sleep()
