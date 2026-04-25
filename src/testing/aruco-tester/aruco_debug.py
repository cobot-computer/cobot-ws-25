import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

class ArucoDebug(Node):
    def __init__(self):
        super().__init__('aruco_debug')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/image_raw', self.cb, 10)
        self.pub = self.create_publisher(Image, '/aruco_debug/image_raw', 10)

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        params = aruco.DetectorParameters()
        params.minMarkerPerimeterRate = 0.01
        detector = aruco.ArucoDetector(dictionary, params)
        corners, ids, _ = detector.detectMarkers(img)
        if ids is not None:
            aruco.drawDetectedMarkers(img, corners, ids)
        self.pub.publish(self.bridge.cv2_to_imgmsg(img, 'rgb8'))

rclpy.init()
rclpy.spin(ArucoDebug())
