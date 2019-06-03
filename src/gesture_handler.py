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

# GESTURE_COMMAND_LIST = [["OPEN", "CLOSED", "OPEN", "COMMAND1"],
                        # ["CLOSED", "OPEN", "CLOSED", "COMMAND2"],
                        # ["LASSO", "OPEN", "CLOSED", "COMMAND3"]]

# Possible gestures: "OPEN", "CLOSED", "LASSO"
GESTURE_COMMAND_LIST = [["OPEN", "CLOSED", "COMMAND1"],
                        ["CLOSED", "OPEN", "COMMAND2"],
                        ["OPEN", "OPEN", "COMMAND3"],
                        ["CLOSED", "CLOSED", "COMMAND4"],
                        ["LASSO", "OPEN", "COMMAND5"],
                        ["LASSO", "CLOSED", "COMMAND6"],
                        ["OPEN", "LASSO", "COMMAND7"],
                        ["CLOSED", "LASSO", "COMMAND8"],
                        ["LASSO", "LASSO", "COMMAND9"]


class gesture_handler():

    def __init__(self, args=None):

        rclpy.init(args=args)

        self.node = rclpy.create_node('gesture_handler')

        self.gesture_msg = Gestures()
        self.gesture_cmd = String()
        self.gesture_seq_pub = String()

        self.i = 0
        self.gesture_counter = 0
        self.timer_period = 0.1
        self.timer_period2 = 0.05
        self.gesture_sequence = []
        self.gesture_collection = [[], [], [], [], [], []]
        self.human_id_list = [0, 1, 2, 3, 4, 5]
        self.sub = self.node.create_subscription(Gestures, '/gestures', self.gestures_callback)
        self.sub2 = self.node.create_subscription(String, '/unification_roscontrol/gestures_sp_to_uni', self.sp_callback)
        self.pub = self.node.create_publisher(String, 'unification_roscontrol/gestures_uni_to_sp')
        self.pub2 = self.node.create_publisher(String, 'gesture_sequence')
        
        self.tmr = self.node.create_timer(self.timer_period, self.timer_callback)
        self.tmr2 = self.node.create_timer(self.timer_period2, self.timer_callback2)

        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()


    def timer_callback(self):
        if len(self.gesture_sequence) == 3:
            if self.gesture_sequence[2] == "CLOSED":
                if any(self.gesture_sequence[:2] == self.i[:2] for self.i in GESTURE_COMMAND_LIST):
                    self.gesture_cmd.data = self.i[2]
                    self.pub.publish(self.gesture_cmd)
                    self.pub2.publish(self.gesture_cmd)
                else:
                    self.gesture_cmd.data = "CLEARED"
                    self.pub.publish(self.gesture_cmd)
                    self.pub2.publish(self.gesture_cmd)
                    self.gesture_sequence.clear()
            else:
                self.gesture_sequence.clear()
        else:
            self.gesture_cmd.data = "NOGESTURE"
            self.pub.publish(self.gesture_cmd)
            

    def sp_callback(self, data):
        if data.data == "CLEAR":
            self.gesture_sequence.clear()
            #self.gesture
        else:
            pass


    def timer_callback2(self):
        self.gesture_seq_pub.data = str(self.gesture_sequence)
        self.pub2.publish(self.gesture_seq_pub)

    
    def gestures_callback(self, gestures):
        if len(self.gesture_collection[gestures.human_id]) < 30:
            self.gesture_collection[gestures.human_id].append(gestures.left_hand)
            self.gesture_collection[gestures.human_id].append(gestures.right_hand)
        elif len(self.gesture_collection[gestures.human_id]) >= 30:
            if all(self.gesture_collection[gestures.human_id][i] == 'open' for i in range(0, len(self.gesture_collection[gestures.human_id]))):
                self.gesture_collection[gestures.human_id].clear()
                if len(self.gesture_sequence) < 3:
                    self.gesture_sequence.append("OPEN")
                else:
                    pass
            elif all(self.gesture_collection[gestures.human_id][i] == 'closed' for i in range(0, len(self.gesture_collection[gestures.human_id]))):
                self.gesture_collection[gestures.human_id].clear()
                if len(self.gesture_sequence) < 3:
                    self.gesture_sequence.append("CLOSED")
                else:
                    pass
            elif all(self.gesture_collection[gestures.human_id][i] == 'lasso' for i in range(0, len(self.gesture_collection[gestures.human_id]))):
                self.gesture_collection[gestures.human_id].clear()
                if len(self.gesture_sequence) < 3:
                    self.gesture_sequence.append("LASSO")
                else:
                    pass
            else:
                self.gesture_collection[gestures.human_id].clear()
            
            #print(self.gesture_collection)
            #self.gesture_seq_pub.data = str(self.gesture_sequence)
            #self.pub2.publish(self.gesture_seq_pub)
            #print(self.gesture_seq_pub.data)
               
        else:
            pass


if __name__ == '__main__':
    try:
        gesture_handler()
    except KeyboardInterrupt:
        pass