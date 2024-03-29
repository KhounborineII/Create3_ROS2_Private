import runner
import sys
import time
import rclpy

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.msg import IrIntensityVector
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

class SonarBot1(runner.HdxNode):
    def __init__(self, namespace: str = ""):
        super().__init__('wheel_publisher')
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.buttons = self.create_subscription(InterfaceButtons, namespace + '/interface_buttons', self.button_callback, qos_profile_sensor_data)
        self.subscription = self.create_subscription(IrIntensityVector, namespace + '/ir_intensity', self.ir_callback, qos_profile_sensor_data)
        self.location = self.create_subscription(Odometry, namespace + '/odom', self.odom_callback, qos_profile_sensor_data)
        timer_period = 0.25 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Required for "Type Anything to Quit" from runner.py"
        self.forward = Twist()
        self.forward.linear.x = 0.5
        self.turn_left = Twist()
        self.turn_left.angular.z = 1.0
        self.turn_right = Twist()
        self.turn_right.angular.z = -1.0
        self.stop = Twist()
        self.stop.linear.x = 0.0
        self.ir_clear_count = 0
        self.irs = set()

    def timer_callback(self):
        self.record_first_callback()

    def button_callback(self, msg: InterfaceButtons):
        if msg.button_1.is_pressed or msg.button_2.is_pressed or msg.button_power.is_pressed:
            self.quit()
            
    def ir_callback(self, msg):
        print(f"IR callback at {self.elapsed_time()}")
        for reading in msg.readings:
            det = reading.header.frame_id
            val = reading.value
            if det != "base_link":
                self.ir_check(det, val)

    def ir_check(self, sensor: str = "", val: int = 0):
        if val > 100:
            self.publisher.publish(self.stop)

    def odom_callback(self, msg):
        for reading in msg.readings:
            loc = reading.pose.pose.position.y
            dir = reading.pose.pose.orientation.z
            if loc > 1500.0:
                while dir > -0.8:
                    self.publisher.publish(self.turn_left)
            elif loc < 0.0:
                while dir < 0.5:
                    self.publisher.publish(self.turn_left)
            else:
                self.publisher.publish(self.forward)


if __name__ == '__main__':
    print(f"Starting up {sys.argv[1]}...")
    runner.run_single_node(lambda: SonarBot1(f'/{sys.argv[1]}'))
