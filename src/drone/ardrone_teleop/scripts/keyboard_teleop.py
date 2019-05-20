#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("ardrone_teleop")
from geometry_msgs.msg import Twist

from os import system

# Library for handling keyboard inputs
from pynput.keyboard import Key, KeyCode, Listener


# Dictionary with changes to attriburtes of Twist Parameter
keys_attrs = {
    KeyCode.from_char("w"): ("linear", "x", 1),
    KeyCode.from_char("s"): ("linear", "x", -1),
    KeyCode.from_char("a"): ("linear", "y", 1),
    KeyCode.from_char("d"): ("linear", "y", -1),
    Key.up: ("linear", "z", 1),
    Key.down: ("linear", "z", -1),
    Key.left: ("angular", "z", 1),
    Key.right: ("angular", "z", -1)
}


# Keys to takeoff and land
key_takeoff = Key.space
key_land = KeyCode.from_char("z")

# Keys to shut down
keys_shutdown = {KeyCode.from_char("c"), Key.ctrl}


# On press keys event handler
def on_press(key, pub, twist, current_keys):
    current_keys.add(key)
    if all([key in keys_shutdown for key in current_keys]):
        return False    # Shutting down
    if key == key_takeoff:
        system("rostopic pub -1 /ardrone/takeoff std_msgs/Empty")
        return True     # Taking off
    elif key == key_land:
        system("rostopic pub -1 /ardrone/land std_msgs/Empty")
        return True     # Landing 

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
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
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

