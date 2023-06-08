import time
import rclpy
import threading
from rclpy.action import ActionClient
from rclpy.node import Node

from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import RotateAngle


class DriveDistanceActionClient(Node):
    '''
    An example of an action client that will cause the iRobot 
    Create3 to drive a specific distance. Subclass of Node.
    '''

    def __init__(self):
        '''
        Purpose
        -------
        initialized by calling the Node constructor, naming our node 
        'drive_distance_action_client'
        '''
        super().__init__('drive_distance_action_client')
        self._action_client = ActionClient(
            self, DriveDistance, 'sena/drive_distance')
        
        self.first_callback_time = None

    def send_goal(self, distance=1.0, max_translation_speed=0.5):
        '''
        Purpose
        -------
        This method waits for the action server to be available, then sends a 
        goal to the server. It returns a future that we can later wait on.
        '''
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = distance
        goal_msg.max_translation_speed = max_translation_speed
        print("Wait 1")
        self._action_client.wait_for_server()
        print("Send Message (1)")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        '''
        Purpose
        -------
        A callback that is executed when the future is complete.
        The future is completed when an action server accepts or rejects the goal request.
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        if self.first_callback_time is None:
            self.first_callback_time = self.elapsed_time()
            print(f"ROS2 activated at {self.first_callback_time}")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        '''
        Purpose
        -------
        Similar to sending the goal, we will get a future that will complete when the result is ready.
        '''
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        #rclpy.shutdown()
    

    def ros_issuing_callbacks(self):
        return self.first_callback_time is not None

class RotateActionClient(Node):
    '''
    An example of an action client that will cause the iRobot 
    Create3 to turn a specific angle. Subclass of Node.
    '''

    def __init__(self):
        '''
        Purpose
        -------
        initialized by calling the Node constructor, naming our node 
        'drive_distance_action_client'
        '''
        super().__init__('rotate_action_client')
        self._action_client = ActionClient(self, RotateAngle, 'sena/rotate_angle')

        self.first_callback_time = None

    def send_goal(self, angle=3.3, max_rotation_speed=0.5):
        '''
        Purpose
        -------
        This method waits for the action server to be available, then sends a 
        goal to the server. It returns a future that we can later wait on.
        '''
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle 
        goal_msg.max_rotation_speed = max_rotation_speed
        print("Wait 2")
        self._action_client.wait_for_server()
        print("Send Message (2)")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        '''
        Purpose
        -------
        A callback that is executed when the future is complete.
        The future is completed when an action server accepts or rejects the goal request.
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        if self.first_callback_time is None:
            self.first_callback_time = self.elapsed_time()
            print(f"ROS2 activated at {self.first_callback_time}")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        '''
        Purpose
        -------
        Similar to sending the goal, we will get a future that will complete when the result is ready.
        '''
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        #rclpy.shutdown()

    def ros_issuing_callbacks(self):
        return self.first_callback_time is not None
    

def main(args=None):
    rclpy.init(args=args)

    action_client = DriveDistanceActionClient()
    action_client2 = RotateActionClient()

    dist  = 1.0
    angle = 1.5
    speed = 0.5

    action_client.send_goal(dist, speed)
    action_client2.send_goal(angle,speed)
    try:
        print("Try Spinning")
        rclpy.spin(action_client)
        rclpy.spin(action_client2)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        rclpy.shutdown()


def drive_thread(finished, ros_ready):
    print("starting")
    rclpy.init(args=None)
    print("init done")
    dist  = 1.0
    speed = 0.5

    driving = DriveDistanceActionClient()
    print("node set up; awaiting ROS2 startup...")
    executor = rclpy.get_global_executor()
    executor.add_node(driving.send_goal(dist, speed))
    while executor.context.ok() and not finished.is_set():
        if driving.ros_issuing_callbacks():
            ros_ready.set()
    rclpy.shutdown()

def spin_thread(finished, ros_ready):
    print("starting")
    rclpy.init(args=None)
    print("init done")
    angle = 3.3
    speed = 0.5

    spinner = RotateActionClient()
    print("node 2 set up; awaiting ROS2 startup...")
    executor = rclpy.get_global_executor()
    executor.add_node(spinner.send_goal(angle, speed))
    while executor.context.ok() and not finished.is_set():
        if spinner.ros_issuing_callbacks():
            ros_ready.set()
    rclpy.shutdown()


def input_thread(finished, ros_ready):
    ros_ready.wait()
    user = input("Type anything to exit")
    finished.set()


if __name__ == '__main__':
    finished = threading.Event()
    ros_ready = threading.Event()
    
    dt = threading.Thread(target=drive_thread, args=(finished,ros_ready))
    st = threading.Thread(target=spin_thread, args=(finished, ros_ready))
    it = threading.Thread(target=input_thread, args=(finished,ros_ready))
    it.start()
    dt.start()
    it.join()
    dt.join()
    st.start()
    st.join()