from robot_interfaces.srv import RobotCommand

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from .servo import Servo

class RobotService(Node):
    
    def __init__(self):
        super().__init__('robot_service')
        self.srv = self.create_service(RobotCommand, 'robot_command', self.robot_command_callback)

        self.servo = Servo()
        self.servo.MOT_R_1.value = 0
        self.servo.MOT_R_2.value = 0
        self.servo.MOT_L_1.value = 0
        self.servo.MOT_L_2.value = 0
        self.servo.drive()

    def robot_command_callback(self, request, response):
        response.done = False
        self.servo.init_variables_L()
        self.servo.init_variables_R()
        self.servo.set_speed(0.3, 0.3)
        while self.servo.count_R < request.dist:
            print(str(self.servo.count_R) + " " + str(self.servo.count_L))
            pass
        self.servo.set_speed(0, 0)
        self.servo.MOT_R_1.value = 0
        self.servo.MOT_R_2.value = 0
        self.servo.init_variables_R()
        self.servo.MOT_L_1.value = 0
        self.servo.MOT_L_2.value = 0
        self.servo.init_variables_L()
        response.done = True
        return response

def main(args=None):
    
    try:
        with rclpy.init(args=args):
            robot_service = RobotService()
            rclpy.spin(robot_service)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
