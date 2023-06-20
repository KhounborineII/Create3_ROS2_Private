# Based on https://github.com/paccionesawyer/Create3_ROS2_Intro/blob/main/individual_examples/pub_LED.py

import runner

import sys
import time
import rclpy

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector

class SonarBot1(runner.HdxNode):
    def __init__(self, namespace: str = ""):
        super().__init__('wheel_publisher')
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.buttons = self.create_subscription(InterfaceButtons, namespace + '/interface_buttons', self.button_callback, qos_profile_sensor_data)
        self.subscription = self.create_subscription(IrIntensityVector, namespace + '/ir_intensity', self.ir_callback, qos_profile_sensor_data)
        self.bumpers = self.create_subscription(HazardDetectionVector, namespace + '/hazard_detection', self.bumper_callback, qos_profile_sensor_data)

        timer_period = 0.25 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Required for "Type Anything to Quit" from runner.py"

        self.forward = Twist()
        self.forward.linear.x = 0.5
        self.backward = Twist()
        self.backward.linear.x = -0.5
        self.turn_left = Twist()
        self.turn_left.angular.z = 1.0
        self.turn_right = Twist()
        self.turn_right.angular.z = -1.0
        self.ir_clear_count = 0
        self.bump_clear = True

    def timer_callback(self):
        self.record_first_callback()

    def button_callback(self, msg: InterfaceButtons):
        if msg.button_1.is_pressed or msg.button_2.is_pressed or msg.button_power.is_pressed:
            self.quit()
            
    def ir_callback(self, msg):
        for reading in msg.readings:
            det = reading.header.frame_id
            val = reading.value
            if det != "base_link":
                self.ir_check(det, val)
        if self.ir_clear_count == 7 and self.bump_clear:
           self.publisher.publish(self.forward)
        self.ir_clear_count = 0

    def ir_check(self, sensor: str = "", val: int = 0):
        if val > 100:
            if sensor.endswith("right"):
                self.publisher.publish(self.turn_left)
            elif sensor.endswith("left"):
                self.publisher.publish(self.turn_right)
            print(sensor)
        else:
            self.ir_clear_count += 1

    def bumper_callback(self, msg):
        for bumper in msg.detections:
            det = bumper.header.frame_id
            self.bumper_check(det)
        self.bump_clear = True
        

    def bumper_check(self, bumper: str = ""):
        if bumper != "base_link":
            self.bump_clear = False
            self.publisher.publish(self.backward)
            if bumper.endswith("right"):
                self.publisher.publish(self.turn_left)
            elif bumper.endswith("left"):
                self.publisher.publish(self.turn_right)
            else:
                self.publisher.publish(self.turn_right)
            print(bumper)


if __name__ == '__main__':
    runner.run_single_node(lambda: SonarBot1('/sena'))
# Change the line above to your robot name here ^ so that it is yours