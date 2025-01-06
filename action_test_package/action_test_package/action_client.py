import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import Int8
from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self.subscriber = self.create_subscription(Int8, 'action_flag', self.action_flag_callback, 10)
        self.goal_handle = None  # goal_handle을 클래스 속성으로 초기화
        self.result_future = None
        self.check = 0
        self.command_flag = False
        self.cancel_flag = False
        self.get_logger().info('Fibonacci action client has been started.')

    def action_flag_callback(self, msg):
        if msg.data == 1:
            if self.goal_handle is not None:  # goal_handle이 유효한지 확인
                if not self.cancel_flag:
                    self.cancelTask()
                    self.cancel_flag = True
            else:
                self.get_logger().info('No active goal to cancel.')
        elif msg.data == 2:
            if not self.command_flag:
                self.send_goal(10)
                self.command_flag = True
            else:
                self.get_logger().info('Already sent goal request...')

    def send_goal(self, order):
        while not self._action_client.wait_for_server():
            self.get_logger().info("'Fibonacci' action server not available, waiting...")

        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, 
                                                                     feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        self.command_flag = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))
    
    def cancelTask(self):
        """Cancel pending task request of any type."""
        if self._get_result_future:
            self.info('Canceling current task.')
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done)
            # rclpy.spin_until_future_complete(self, future)

    def cancel_done(self, future):
        self.info('Canceling task done.')
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.info("Goal successfully canceled")
        else:
            self.info("Goal failed to cancel")
        self.cancel_flag = False

    def info(self, msg):
        self.get_logger().info(msg)

    def warn(self, msg):
        self.get_logger().warn(msg)

    def error(self, msg):
        self.get_logger().error(msg)

    def debug(self, msg):
        self.get_logger().debug(msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()
    # while rclpy.ok():
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
