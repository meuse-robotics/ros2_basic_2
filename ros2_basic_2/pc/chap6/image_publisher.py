import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
#from cv2 import aruco
 
class ImagePublisher(Node):
  
  def __init__(self):
    super().__init__('image_publisher')
    self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.cap = cv2.VideoCapture(2)
    self.br = CvBridge()

    ### --- aruco設定 --- ###
    #self.dict_aruco = aruco.Dictionary_get(aruco.DICT_4X4_50)
    #self.parameters = aruco.DetectorParameters_create()
   
  def timer_callback(self):
    ret, frame = self.cap.read()
          
    if ret == True:
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
      # エッジ抽出
      #frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      #frame_edge = cv2.Canny(frame_gray, threshold1=100, threshold2=200)"""
      #self.publisher_.publish(self.br.cv2_to_imgmsg(frame_edge))
      # ArUcoマーカー
      #gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
      #corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.dict_aruco, parameters=self.parameters)
      #frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
      #self.publisher_.publish(self.br.cv2_to_imgmsg(frame_markers))
 
    # Display the message on the console
    self.get_logger().info('Publishing video frame')

def main(args=None):
    try:
        with rclpy.init(args=args):
            image_publisher = ImagePublisher()	# ノードを生成
            rclpy.spin(image_publisher) # callback関数が呼ばれる
            
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
  main()