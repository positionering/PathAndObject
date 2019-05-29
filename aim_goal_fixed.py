#!/usr/bin/env python
import math
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

msg_to_publish = Float32MultiArray()

qr = 0 
value = 0


def calcpos(x_GPS, y_GPS, t_gps):
     global value     

     #rotation av gps koordinat system.

     x_gps_diff = float(x_GPS-x_start)
     y_gps_diff = float(y_GPS-y_start)

     x_rel = x_gps_diff*math.cos(-t_start) - y_gps_diff*math.sin(-t_start)
     y_rel = x_gps_diff*math.sin(-t_start) + y_gps_diff*math.cos(-t_start)
     #print(r_gps)

    # rotation bil
     x, y = x_goal-x_rel, y_goal-y_rel
    
     #print(t_gps)
     t_r = t_gps - t_start
     
     xGoal = x*math.cos(-t_r) - y*math.sin(-t_r)
     yGoal = x*math.sin(-t_r) + y*math.cos(-t_r)
     #print(t_r)
    
     

     if yGoal < 0:
        return [0 ,0]
     elif value == 1 and yGoal >0:
        return [xGoal, yGoal]
     else:
        return [0 ,0]

     
def talker():

    pub = rospy.Publisher('aim_goal', Float32MultiArray , queue_size=2)
    while not rospy.is_shutdown():
        msg_to_publish.data = NEW_GOAL
        pub.publish(msg_to_publish)
	print(NEW_GOAL)
	break
        


def callback(data):
    global GPS_POS
    GPS_POS = data.data
    global NEW_GOAL 
    if qr ==1:
    	NEW_GOAL = calcpos(GPS_POS[0], GPS_POS[1], GPS_POS[2])
    	talker()

def callback2(data):
    aim_POS = data.data
    
    print(aim_POS)
    if aim_POS == "stop":
        global value 
        value = 0
    
    else:
    
        array = aim_POS.split(",")
        global x_goal  
        global y_goal 
        x_goal = float(array[0])/10
        y_goal = float(array[1])/10
        print(x_goal)
        print(y_goal)


        global x_start
        global y_start
        global t_start
       
        x_start = GPS_POS[0]
        y_start = GPS_POS[1]
        t_start = GPS_POS[2]
        
        global value
        value = 1

        global qr
        qr = 1


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('NEWGOAL', anonymous=True)

    rospy.Subscriber('GPS_pos', Float32MultiArray, callback)

    rospy.Subscriber('aim_coords', String, callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
	






