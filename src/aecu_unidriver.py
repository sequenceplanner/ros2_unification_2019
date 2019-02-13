#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres, Martin Dahl
    # AECU ROS2 Unification Driver
    # V.0.1.0.
#----------------------------------------------------------------------------------------

import sys
import rclpy
import time
from std_msgs.msg import String
from unification_ros2_messages.msg import AecuSPToUni
from unification_ros2_messages.msg import AecuUniToSP
import time

# Emulator GPIO's
import GPIOEmu as GPIO  

# Real GPIO's
#import RPi.GPIO as GPIO

class aecu_unidriver():

    def __init__(self, args=None):

        rclpy.init(args=args)

        self.node = rclpy.create_node('aecu_unidriver')
        
        #self.lf_tool_home = False
        #self.filter_tool_home = False

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

        # state
        self.tool_is_idle = False
        self.tool_is_running_forward = False
        self.tool_is_running_reverse = False
        self.positioned_at_home_station = False
        self.operating_position = False
        self.pre_home_position = False
        self.unclear_position = False
        self.programmed_torque_reached = False

        # command
        self.set_tool_idle = False
        self.run_tool_forward = False
        self.run_tool_in_reverse = False
        self.inhibit_all_run_also_manual = False
        self.activate_unload = False
        self.activate_lift = False

        #??
        self.on_timer = 0
        self.off_timer = 0
        self.manualRun = False

        self.pub = self.node.create_publisher(AecuUniToSP, '/unification_roscontrol/aecu_uni_to_sp')
        self.sub = self.node.create_subscription(AecuSPToUni, '/unification_roscontrol/aecu_sp_to_uni', self.spCallback)
        timer_period = 0.01
        self.tmr = self.node.create_timer(timer_period, self.timer_callback)

        rclpy.spin(self.node)

        self.node.destroy_node()
        rclpy.shutdown()


    def timer_callback(self):

        msg = AecuUniToSP()

        msg.got_cmd_set_tool_idle = self.set_tool_idle
        msg.got_cmd_run_tool_forward = self.run_tool_forward
        msg.got_cmd_run_tool_in_reverse = self.run_tool_in_reverse
        msg.got_cmd_inhibit_all_run_also_manual = self.inhibit_all_run_also_manual
        msg.got_cmd_activate_unload = self.activate_unload
        msg.got_cmd_activate_lift = self.activate_lift
        msg.tool_is_idle = self.tool_is_idle
        msg.tool_is_running_forward = self.tool_is_running_forward
        msg.tool_is_running_reverse = self.tool_is_running_reverse
        msg.positioned_at_home_station = self.positioned_at_home_station
        msg.operating_position = self.operating_position
        msg.pre_home_position = self.pre_home_position
        msg.unclear_position = self.unclear_position
        msg.programmed_torque_reached = self.programmed_torque_reached

        self.manualRun = (GPIO.input(self.GPI1) == 1)

        if GPIO.input(self.GPI4) == 1:
                self.tool_is_in_alarm = True
        else:
            self.tool_is_in_alarm = False
        
        # Read atlas tool position
        if GPIO.input(self.GPI5) == 1 or\
            (GPIO.input(self.GPI7) == 1 and\
            GPIO.input(self.GPI6) == 0):
            self.positioned_at_home_station = False
            self.operating_position = False
            self.pre_home_position = False
            self.unclear_position = True

        elif GPIO.input(self.GPI6) == 1 and\
            GPIO.input(self.GPI7) == 1:
            self.positioned_at_home_station = True
            self.operating_position = False
            self.pre_home_position = False
            self.unclear_position = False
        
        elif GPIO.input(self.GPI6) == 1 and\
            GPIO.input(self.GPI7) == 0:
            self.positioned_at_home_station = False
            self.operating_position = False
            self.pre_home_position = True
            self.unclear_position = False
        
        else:
            self.positioned_at_home_station = False
            self.operating_position = True
            self.pre_home_position = False
            self.unclear_position = False
        
        # Read torque
        if GPIO.input(self.GPI3) == True:
            self.programmed_torque_reached = True
        
        else:
            self.programmed_torque_reached = False

        # Not sure what this is
        if GPIO.input(self.GPI7) == True:
            self.tool_is_idle = True
        
        else:
            self.tool_is_idle = True
    
        self.pub.publish(msg)


    def aecu_set_tool_idle(self):
        GPIO.output(self.GPO1, 0)
        GPIO.output(self.GPO2, 0)

    def aecu_run_tool_forward(self):
        GPIO.output(self.GPO1, 1)                 
        GPIO.output(self.GPO2, 0) 

    def aecu_run_tool_in_reverse(self):
        GPIO.output(self.GPO1, 1)
        GPIO.output(self.GPO2, 1)

    def aecu_inhibit_all_run_also_manual(self):
        GPIO.output(self.GPO1, 0)
        GPIO.output(self.GPO2, 0)

    def aecu_activate_unload(self):
        GPIO.output(self.GPO3, 1)
        GPIO.output(self.GPO4, 1)

    def aecu_disable_unload(self):
        # What outputs?
        pass

    def aecu_activate_lift(self):
        GPIO.output(self.GPO3, 1)
        GPIO.output(self.GPO4, 0)

    def aecu_disable_lift(self):
        # What outputs?
        pass

    def spCallback(self, aecu_cmd):

        self.set_tool_idle = aecu_cmd.set_tool_idle
        self.run_tool_forward = aecu_cmd.run_tool_forward
        self.run_tool_in_reverse = aecu_cmd.run_tool_in_reverse
        self.inhibit_all_run_also_manual = aecu_cmd.inhibit_all_run_also_manual
        self.activate_unload = aecu_cmd.activate_unload
        self.activate_lift = aecu_cmd.activate_lift

        if self.manualRun:
            self.aecu_run_tool_forward()

        if self.set_tool_idle == True and\
            self.run_tool_forward == False and\
            self.run_tool_in_reverse == False and\
            self.inhibit_all_run_also_manual == False:
            self.aecu_set_tool_idle()

        elif self.set_tool_idle == False and\
            self.run_tool_forward == True and\
            self.run_tool_in_reverse == False and\
            self.inhibit_all_run_also_manual == False:
            self.aecu_run_tool_forward()

        elif self.set_tool_idle == False and\
            self.run_tool_forward == False and\
            self.run_tool_in_reverse == True and\
            self.inhibit_all_run_also_manual == False:
            self.aecu_run_tool_in_reverse()
        
        elif self.set_tool_idle == False and\
            self.run_tool_forward == False and\
            self.run_tool_in_reverse == False and\
            self.inhibit_all_run_also_manual == True:
            self.aecu_inhibit_all_run_also_manual()

        else:
            self.aecu_set_tool_idle()

        
        if self.activate_lift == True:
            self.aecu_activate_lift()
        else:
            self.aecu_disable_lift()

        
        if self.activate_unload == True:
            self.aecu_activate_unload()
        else:
            self.aecu_disable_unload()
       

if __name__ == '__main__':
    aecu_unidriver()