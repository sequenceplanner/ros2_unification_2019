#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # ROS2 Kinect Gesture Sequence Handler and Publisher
    # V.1.0.0.
#----------------------------------------------------------------------------------------

import sys
import rclpy
import time
from std_msgs.msg import String
from unification_ros2_messages.msg import Gestures
#from unification_ros2_messages.msg import GesturesUniToSP
#from unification_ros2_messages.msg import GesturesSPToUni

GESTURE_COMMAND_LIST = [["OPEN", "CLOSED", "OPEN", "COMMAND1"],
                        ["CLOSED", "OPEN", "CLOSED", "COMMAND2"],
                        ["LASSO", "OPEN", "CLOSED", "COMMAND3"]]


class gesture_handler():

    def __init__(self, args=None):

        rclpy.init(args=args)

        self.gesture_msg = Gestures()
        self.gesture_cmd = String()
        self.node = rclpy.create_node('gesture_handler')

        self.i = 0
        self.gesture_counter = 0
        self.timer_period = 0.1
        self.gesture_sequence = []
        self.gesture_collection = []
        self.sub = self.node.create_subscription(Gestures, '/gestures', self.gestures_callback)
        self.sub = self.node.create_subscription(String, '/unification_roscontrol/gestures_sp_to_uni', self.sp_callback)
        self.pub = self.node.create_publisher(String, 'unification_roscontrol/gestures_uni_to_sp')
        
        self.tmr = self.node.create_timer(self.timer_period, self.timer_callback)

        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()


    def timer_callback(self):
        if len(self.gesture_sequence) == 4:
            if self.gesture_sequence[3] == "LASSO":
                if any(self.gesture_sequence[:3] == self.i[:3] for self.i in GESTURE_COMMAND_LIST):
                    self.gesture_cmd.data = self.i[3]
                    self.pub.publish(self.gesture_cmd)
                else:
                    self.gesture_cmd.data = "NOTSPECIFIED"
                    self.pub.publish(self.gesture_cmd)
                    self.gesture_sequence.clear()
            else:
                self.gesture_sequence.clear()
        else:
            self.gesture_cmd.data = "NOGESTURE"
            self.pub.publish(self.gesture_cmd)
            

    def sp_callback(self, data):
        if data.data == "CLEAR":
            self.gesture_sequence.clear()
        else:
            pass

    
    def gestures_callback(self, gestures):
        if len(self.gesture_collection) < 30:
            self.gesture_collection.append(gestures.left_hand)
            self.gesture_collection.append(gestures.right_hand)
        elif len(self.gesture_collection) >= 30:
            if all(self.gesture_collection[i] == 'open' for i in range(0, len(self.gesture_collection))):
                self.gesture_collection.clear()
                if len(self.gesture_sequence) < 4:
                    self.gesture_sequence.append("OPEN")
                else:
                    pass
            elif all(self.gesture_collection[i] == 'closed' for i in range(0, len(self.gesture_collection))):
                self.gesture_collection.clear()
                if len(self.gesture_sequence) < 4:
                    self.gesture_sequence.append("CLOSED")
                else:
                    pass
            elif all(self.gesture_collection[i] == 'lasso' for i in range(0, len(self.gesture_collection))):
                self.gesture_collection.clear()
                if len(self.gesture_sequence) < 4:
                    self.gesture_sequence.append("LASSO")
                else:
                    pass
            else:
                self.gesture_collection.clear()
            
            print(self.gesture_sequence)
               
        else:
            pass


if __name__ == '__main__':
    try:
        gesture_handler()
    except KeyboardInterrupt:
        pass