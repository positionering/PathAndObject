#!/usr/bin/env python
import math
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

msg_to_publish = Float32MultiArray()

x_goal = 0
y_goal = 10

qr = 0

def calcpos(x_GPS, y_GPS, t_gps):
     global qr
     global x_start
     global y_start
     global t_start
     if qr == 0:
         x_start = x_GPS
         y_start = y_GPS
         t_start = t_gps
	# print('hello')
         qr = 1
     #print(t_start)

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
    
     return [xGoal, yGoal]

     
def talker():

    pub = rospy.Publisher('relative_goal', Float32MultiArray , queue_size=1)
    while not rospy.is_shutdown():
        msg_to_publish.data = NEW_GOAL
        pub.publish(msg_to_publish)
	print(NEW_GOAL)
	break
        


def callback(data):
    GPS_POS = data.data
    global NEW_GOAL 
    NEW_GOAL = calcpos(GPS_POS[0], GPS_POS[1], GPS_POS[2])
   
    talker()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('NEWGOAL', anonymous=True)

    rospy.Subscriber('GPS_pos', Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
	







