#!/usr/bin/python

import rospy
import roslib; roslib.load_manifest("car_teleop")
from geometry_msgs.msg import Twist


import curses
import os

import signal
import sys

signal.signal(signal.SIGINT, lambda sig, frame: sys.exit(0))


pub = rospy.Publisher("car/cmd_vel", Twist, queue_size=1)
rospy.init_node("keyboard_teleop", anonymous=True)
twist = Twist()


def limit(value, mini, maxi):
    return value if value <= maxi and value >= mini else maxi if value > maxi else mini 


def main(win):
    win.nodelay(True)
    key=""
    win.clear()  
    rate = rospy.Rate(10) # 10hz              
    while not rospy.is_shutdown():         
        try:                 
            key = win.getkey()         
            win.clear()                
            if (str(key) == 'i'):
	            twist.linear.x = limit(twist.linear.x + 15, -15, 15)
            if (str(key) == 'k'):
	            twist.linear.x = limit(twist.linear.x - 15, -15, 15) 
            if (str(key) == 'j'):
	            twist.angular.z = limit(twist.angular.z + 15, -15, 15)
            if (str(key) == 'l'):
	            twist.angular.z = limit(twist.angular.z - 15, -15, 15)  
            pub.publish(twist)                                       
            win.addstr(str(key))          
        except Exception as e:
            pub.publish(twist)   
        rate.sleep()  

 
if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
