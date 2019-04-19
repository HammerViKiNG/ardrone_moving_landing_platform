#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("car_teleop")
from geometry_msgs.msg import Twist

from os import system

# Library for handling keyboard inputs
from pynput.keyboard import Key, KeyCode, Listener


# Dictionary with changes to attriburtes of Twist Parameter
keys_attrs = {
    KeyCode.from_char("i"): ("linear", "x", 3),
    KeyCode.from_char("k"): ("linear", "x", -3),
    KeyCode.from_char("j"): ("angular", "z", 1),
    KeyCode.from_char("l"): ("angular", "z", -1)
}


# Keys to shut down
keys_shutdown = {KeyCode.from_char("c"), Key.ctrl}


# On press keys event handler
def on_press(key, pub, twist, current_keys):
    current_keys.add(key)
    if all([key in keys_shutdown for key in current_keys]):
        return False    # Shutting down

    # Ordinar keybord inpputs handling
    if key in keys_attrs:
        setattr(getattr(twist, keys_attrs[key][0]), keys_attrs[key][1], keys_attrs[key][2])
    pub.publish(twist)
 

# on release keys event handler
def on_release(key, pub, twist, current_keys):
    current_keys.remove(key)
    if key in keys_attrs:
        setattr(getattr(twist, keys_attrs[key][0]), keys_attrs[key][1], 0)
    pub.publish(twist)


# ROS and keyboard handler initializition initialization 
def initialize():
    pub = rospy.Publisher("car/cmd_vel", Twist, queue_size=1)
    rospy.init_node("keyboard_teleop", anonymous=True)
    twist = Twist()
    current_keys = set()

    with Listener(
        on_press=lambda key: on_press(key, pub, twist, current_keys),
        on_release=lambda key: on_release(key, pub, twist, current_keys)
        ) as listener:
            listener.join()

 
if __name__ == '__main__':
     try:
         initialize()
     except rospy.ROSInterruptException:
         pass

