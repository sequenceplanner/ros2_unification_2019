#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres, Martin Dahl
    # HECU ROS2 Unification Driver
    # V.0.1.0.
#----------------------------------------------------------------------------------------

import sys
import rclpy
import time
from std_msgs.msg import String
from unification_ros2_messages.msg import HecuUniToSP

# Emulator GPIO's
import GPIOEmu as GPIO  

# Real GPIO's
#import RPi.GPIO as GPIO

class hecu_unidriver():

    def __init__(self, args=None):

        rclpy.init(args=args)

        self.node = rclpy.create_node('hecu_unidriver')
        
        self.lf_tool_home = False
        self.filter_tool_home = False

        self.GPO1 = 4
        self.GPO2 = 17
        self.GPO3 = 18
        self.GPO4 = 27
        self.GPI1 = 5
        self.GPI2 = 6
        self.GPI3 = 12
        self.GPI4 = 13
        self.GPI5 = 16
        self.GPI6 = 19
        self.GPI7 = 22
        self.GPI8 = 23

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)               
        GPIO.setup(self.GPO1, GPIO.OUT)
        GPIO.setup(self.GPO2, GPIO.OUT)
        GPIO.setup(self.GPO3, GPIO.OUT)
        GPIO.setup(self.GPO4, GPIO.OUT)
        GPIO.setup(self.GPI1, GPIO.IN)
        GPIO.setup(self.GPI2, GPIO.IN)
        GPIO.setup(self.GPI3, GPIO.IN)
        GPIO.setup(self.GPI4, GPIO.IN)
        GPIO.setup(self.GPI5, GPIO.IN)
        GPIO.setup(self.GPI6, GPIO.IN)
        GPIO.setup(self.GPI7, GPIO.IN)
        GPIO.setup(self.GPI8, GPIO.IN)

        GPIO.output(self.GPO1, False)
        GPIO.output(self.GPO2, False)
        GPIO.output(self.GPO3, False)
        GPIO.output(self.GPO4, False)

        self.pub = self.node.create_publisher(HecuUniToSP, '/unification_roscontrol/hecu_uni_to_sp')
        timer_period = 1.0
        self.tmr = self.node.create_timer(timer_period, self.timer_callback)

        rclpy.spin(self.node)

        self.node.destroy_node()
        rclpy.shutdown()


    def timer_callback(self):

        msg = HecuUniToSP()
        msg.lf_tool_home = self.lf_tool_home
        msg.filter_tool_home = self.filter_tool_home
    
        if GPIO.input(self.GPI1) == 1:
            self.lf_tool_home = True
        else:
            self.lf_tool_home = False   
        if GPIO.input(self.GPI2) == 1:
            self.filter_tool_home = True
        else:
            self.filter_tool_home = False   
            
        self.pub.publish(msg)
       

if __name__ == '__main__':
    hecu_unidriver()
