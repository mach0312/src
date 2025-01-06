import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('Fibonacci action server has been started.')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        self.get_logger().info('Generating Fibonacci sequence...')
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:  # 속성으로 참조
                goal_handle.canceled()
                self.get_logger().info('Action canceled...')
                return Fibonacci.Result()  # 빈 결과 반환
            
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
    
    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request: {goal_request}')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, cancel_request):
        self.get_logger().info(f'Canceling goal... ')
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()
    excutor = MultiThreadedExecutor()

    rclpy.spin(fibonacci_action_server,executor=excutor)


if __name__ == '__main__':
    main()
