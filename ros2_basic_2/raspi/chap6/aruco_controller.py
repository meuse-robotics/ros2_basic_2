import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from .servo import Servo
#cv
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from cv2 import aruco

class ControllerNode(Node):
    
    def __init__(self):
        super().__init__('controller_node')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription

        self.servo = Servo()
        self.servo.MOT_R_1.value = 0
        self.servo.MOT_R_2.value = 0
        self.servo.MOT_L_1.value = 0
        self.servo.MOT_L_2.value = 0
        self.servo.drive()

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()
        ### --- aruco設定 --- ###
        self.dict_aruco = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
   
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.dict_aruco, parameters=self.parameters)
            frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
            
            if ids is not None:
                if ids[0]==0:
                    self.vel.linear.x = 0.3
                    self.vel.angular.z = 0.0
                elif ids[0]==1:
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 0.3
                elif ids[0]==2:
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = -0.3
                elif ids[0]==3:
                    self.vel.linear.x = -0.3
                    self.vel.angular.z = 0.0
                elif ids[0]==4:
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 0.0
                self.publisher_.publish(self.vel)
                        
    def listener_callback(self, Twist):
        self.get_logger().info(f'並進速度={Twist.linear.x}角速度={Twist.angular.z}')
        self.target_speed_R = Twist.linear.x + Twist.angular.z
        self.target_speed_L = Twist.linear.x - Twist.angular.z
        self.servo.set_speed(self.target_speed_L, self.target_speed_R)
        
def main(args=None):
    
    try:
        with rclpy.init(args=args):
            controller_node = ControllerNode()
            rclpy.spin(controller_node)
        
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()
