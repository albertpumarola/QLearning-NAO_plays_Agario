#!/usr/bin/env python

from agario_mouse.srv import *
import rospy
from pymouse import PyMouse
from pykeyboard import PyKeyboard

mouse = PyMouse()
keyboard = PyKeyboard()

def handle_agario_mouse(req):
    mouse.move(req.x, req.y)
    return 1

def agario_mouse_server():
    rospy.init_node('agario_mouse')
    s = rospy.Service('agario_mouse', Mouse, handle_agario_mouse)
    rospy.spin()

if __name__ == "__main__":
    agario_mouse_server()
