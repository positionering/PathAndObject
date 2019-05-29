#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

import matplotlib.pyplot as plt 
import numpy as np
import matplotlib.mlab as mlab
###############################################################################

#import numpy as np
#from matplotlib import pyplot
#import time
#from mpl_toolkits.mplot3d import Axes3D
#from ast import literal_eval

###############################################################################
frame_width = 640
frame_height = 480
forward_max = 660  # max distance forward, has to be calibrated

once = False

# Calculate the combined value of every pixels values
def calculate_z(first_byte, second_byte):
    return float(((int(second_byte) << 8) | int(first_byte))) / 65535

# Generate an int array from a simple string
def string_to_int_arr(z_string):
    z_int_arr = []
    space_removed = z_string.replace(' ', '')
    z_string_arr = space_removed.split(',')
    for j in range(frame_height):
        for i in range(frame_width):
            #print("length of z_string_arr " + str(len(z_string_arr)))
            #print("frame_width length " + str((j * frame_width + i) * 2 + 1))
            z_int_arr.append(int(calculate_z(z_string_arr[(j * frame_width + i) * 2],
                                         z_string_arr[(j * frame_width + i) * 2 + 1]) * forward_max))
    return z_int_arr

# Generate an int array from a raw image message
def generate_z_array(image_message):
    message_string = str(image_message)
    #message_string = image_message #commented because of test enviroment
    start = message_string.find("data") + 7
    end = len(message_string) - 1
    z_int_arr = string_to_int_arr(message_string[start:end])
    return z_int_arr

##############################################################################

# scales coord
def coord_transform(pixel_pos, depth_res, rgb_res, depth_ang, rgb_ang):
    V_djup = depth_ang*math.pi/180; #vinkeln bredd pa djup kameran
    V_rgb = rgb_ang*math.pi/180; #vinkeln bredd pa rgb kameran

    d = (math.sin(V_djup/2)-math.tan(V_rgb/2)*math.cos(V_djup/2))/(2*math.sin(V_djup/2));
    k = (depth_res/rgb_res)*(1-2*d);
    m = depth_res * d;
    
    scaled_pixel_pos = round(k*pixel_pos+m)
    return int(scaled_pixel_pos)
    
#calculates the angle for koordtesian transoformation
def kart_angle(scaled_x_top):
    return math.atan((((scaled_x_top-640.0/2.0)/(640.0/2.0))*math.sin(75.0*math.pi/(2.0*180.0)))/math.cos(75.0*math.pi/(180.0*2.0)))


# one_Object_cords is of type "TopcornerXY/BottomcornerXY: x,y,x,y,Object". Returns all the z value of the object as array
def object_z_values(z_int_arr, one_object_cords):
    full_z = []
    object_array = one_object_cords.split(",")  # Generate an array with one objects coords and type

    x_Top = int(object_array[0])
    y_Top = int(object_array[1])
    x_Bot = int(object_array[2])
    y_Bot = int(object_array[3])
    
    scaled_x_top = (coord_transform(x_Top, 640.0, 1920.0, 75.0, 69.4))
    scaled_x_bot = (coord_transform(x_Bot, 640.0, 1920.0, 75.0, 69.4))
    scaled_y_top = (coord_transform(y_Top, 480.0, 1080.0, 65.0, 42.5))
    scaled_y_bot = (coord_transform(y_Bot, 480.0, 1080.0, 65.0, 42.5))
  
    #scaled_x_top = (coord_transform(x_Top, 640.0, 640.0, 75.0, 69.4))
    #scaled_x_bot = (coord_transform(x_Bot, 640.0, 640.0, 75.0, 69.4))
    #scaled_y_top = (coord_transform(y_Top, 480.0, 480.0, 65.0, 42.5))
    #scaled_y_bot = (coord_transform(y_Bot, 480.0, 480.0, 65.0, 42.5))

    for i in range(scaled_y_top, scaled_y_bot+1):  # Assume 0,0 is top left corner for camera
        for j in range(scaled_x_top, scaled_x_bot+1):
            full_z.append(z_int_arr[frame_width * i + j - 1])
            
    return full_z

# Returns a string with coordinates as: "x,z,x,z,x,z" for every point
def object_closest_line(full_z, one_object_cords):
    object_array = one_object_cords.split(",")

    x_Top = int(object_array[0])
    y_Top = int(object_array[1])
    x_Bot = int(object_array[2])
    y_Bot = int(object_array[3])
    
    scaled_x_top = (coord_transform(x_Top, 640.0, 1920.0, 75.0, 69.4))
    scaled_x_bot = (coord_transform(x_Bot, 640.0, 1920.0, 75.0, 69.4))
    scaled_y_top = (coord_transform(y_Top, 480.0, 1080.0, 65.0, 42.5))
    scaled_y_bot = (coord_transform(y_Bot, 480.0, 1080.0, 65.0, 42.5))
    
    #scaled_x_top = (coord_transform(x_Top, 640.0, 640.0, 75.0, 69.4))
    #scaled_x_bot = (coord_transform(x_Bot, 640.0, 640.0, 75.0, 69.4))
    #scaled_y_top = (coord_transform(y_Top, 480.0, 480.0, 65.0, 42.5))
    #scaled_y_bot = (coord_transform(y_Bot, 480.0, 480.0, 65.0, 42.5))
    
    #print("scaled xTop: " + str(scaled_x_top))
    #print("scaled xBot: " + str(scaled_x_bot))
    
    x_diff = scaled_x_bot-scaled_x_top
    y_diff = scaled_y_bot-scaled_y_top
    line = []

    #closest_value = full_z[0]
    #index = 0
    #for i in range(len(full_z) - 1):
    #    if (closest_value < full_z[i + 1]):  # High values are closer
    #        closest_value = full_z[i + 1]
    #        index = i + 1
    
    # 35 depth 
    pixel_sum = 0
    median_array = []
    index = int(y_diff*0.35 * x_diff + x_diff*0.35)
    for k in range (int(y_diff*0.3)): 
        for l in range (int(x_diff*0.3)):
            if full_z[index + l] != 0:
                pixel_sum = pixel_sum + full_z[index + l]
                median_array.append(full_z[index + l])
        index = index + x_diff
    max_pixel_val = pixel_sum/(y_diff*x_diff*0.3*0.3) 
    
    median_array.sort()
    #print("average: " + str(max_pixel_val))
    max_pixel_val = median_array[int(len(median_array)*0.1)]
    #print("median: " + str(max_pixel_val))

    #global once
    #if once == False:     
    #    f = open("35depth.txt","w+")
    #    for i in range(len(median_array)):
    #        f.write(str(median_array[i])+", ")
        #f.write(str(len(full_z)))
    #    f.close()
    #    once = True           
    #global once
    #if once == False:
    #    plt.axis([0,120,0,1000])
    #    n, bins, patches = plt.hist(full_z, bins=100, color = '#0504aa')
    #    plt.grid(axis='y', alpha = 0.75)
    #    maxfreq = n.max()    
        #plt.ylim(top=np.ceil(maxfreq / 10) * 10 if maxfreq % 10 else maxfreq + 10)
    #    once = True
    #    plt.show()
    
       
    #Generate line with the highest z-values   
    #print("depth: " + str(max_pixel_val))
    for j in range (x_diff):
        #print("x1 " + str(max_pixel_val*math.cos(kart_angle(scaled_x_top+j))))
        #print("z1 " + str(max_pixel_val*math.sin(kart_angle(scaled_x_top+j))))
        new_z  = int(round(max_pixel_val*math.cos(kart_angle(scaled_x_top+j))))
        new_x  = int(round(max_pixel_val*math.sin(kart_angle(scaled_x_top+j))))
        #print("x2 " + str(new_x))
        #print("z2 " + str(new_z))
        
        if len(line) == 0 or not int(line[len(line)-4]) == new_x: #make sure not to add second equal x-value         
            
            line.append(str(new_x))
            line.append(",")
            if new_z>10:
                line.append(str(new_z-10))
            else:
                line.append(str(1))
            line.append(",")

    line.append(object_array[4])  # Adding type of object
    line.append(";")
    return ''.join(line)

# Returns a string with all object lines as: "x,z,x,z,type;x,z,x,z,type"
def all_objects_closest_line(image_message, objects_cords):
    full_line = []
    z_int_arr = generate_z_array(image_message)

    #print(len(objects_cords))
    for i in range(len(objects_cords)):
        index_values = objects_cords[i].split(",")
        if(index_values[4] == "person" or index_values[4] == "chair" or index_values[4]=='bench'): #add chair 
            object_z = object_z_values(z_int_arr, objects_cords[i])
            
            full_line.append(object_closest_line(object_z, objects_cords[i]))

    print("")
    return ''.join(full_line)

###############################################################################
# TestEnviroment

#if __name__ == "__main__":
#    object_coordinates = "1,2,2,4,Person;6,4,7,6,Cone"
#    object_array = object_coordinates.split(";")
#    frame_values = "data: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 3, 0, 0, 0, 0, 0, 0, 0, 3, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

#    output = all_objects_closest_line(frame_values, object_array)  # frame_values is of format array of strings
#    print(output)

###############################################################################
#The real enviroment
class Listener:
    #init function
    def __init__(self):
        self.image = None
        self.coords = None

    # The message is converted to array of ints
    def callback_function_image(self, image_message):
        self.image = image_message #Not 100% certain it should be .data
        
        self.compute()

    # The message is co480,nverted to array of several objects
    def callback_function_cord(self, objects_cords):
        self.coords = objects_cords.data.split(";")
        
        self.compute()

    def compute(self):
        if self.image is not None and self.coords is not None:
            if not (self.coords[0] == "empty"):
                objects_z_values = all_objects_closest_line(self.image, self.coords)
                #print("object_z_values: " + str(all_objects_closest_line(self.image, self.coords)))
                print("Output of objects_stream is: " + str(objects_z_values))
                msg_to_publish.data = objects_z_values
                pub.publish(msg_to_publish)
            else: 
                msg_to_publish.data = ""
                print("found nothing")
                pub.publish(msg_to_publish)

            self.image = None
            self.coords = None

# Naming the node and initializing the subscriber
if __name__ == "__main__":
    rospy.init_node("objects_stream")
    myListener = Listener()

    # Defining the publisher (name of output topic, type, msg queue size)
    pub = rospy.Publisher('depth_output', String, queue_size=2)
    msg_to_publish = String()
    # Defining the subscribers (name of topic to collect from, type, function to call on)
    #rospy.Subscriber('/image_data', String, myListener.callback_function_image)
    #rospy.Subscriber('/coordinates_data', String, myListener.callback_function_cord)
    
    rospy.Subscriber('/camera/depth/image_rect_raw', Image, myListener.callback_function_image, queue_size = 1)
    rospy.Subscriber('Yolo_data', String, myListener.callback_function_cord, queue_size = 1)  # Define right topic

    rospy.spin()

###############################################################################
