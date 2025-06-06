import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from .servo import Servo

class CommandSubscriberAction(Node):
    
    def __init__(self):
        super().__init__('command_subscriber_action')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.servo = Servo()
        self.servo.MOT_R_1.value = 0
        self.servo.MOT_R_2.value = 0
        self.servo.MOT_L_1.value = 0
        self.servo.MOT_L_2.value = 0
        self.servo.drive()
                
    def listener_callback(self, Twist):
        self.get_logger().info(f'並進速度={Twist.linear.x}角速度={Twist.angular.z}')
        self.target_speed_R = Twist.linear.x + Twist.angular.z
        self.target_speed_L = Twist.linear.x - Twist.angular.z
        self.servo.set_speed(self.target_speed_L, self.target_speed_R)
        
def main(args=None):
    
    try:
        with rclpy.init(args=args):
            command_subscriber_action = CommandSubscriberAction()
            rclpy.spin(command_subscriber_action)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
