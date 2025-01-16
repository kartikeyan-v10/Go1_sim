#!/usr/bin/python3

import rclpy
import cv2
from cv2 import aruco
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image

class ImageCapture(Node):

    def __init__(self):

        super().__init__('image_capture')           
        self.color_cam_sub = self.create_subscription(Image, '/camera/image_raw', self.cv_bridge, 10)
        self.bridge = CvBridge()
        #image_processing_rate = 0.2
        #self.timer = self.create_timer(image_processing_rate, self.process_image)      
        self.cv_image = None   
        self.ids = []                                                        
        
    
    def cv_bridge(self, msg):
        try:
            # Convert the ROS Image message to OpenCV format
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.process_image()

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {str(e)}')


    def process_image(self):

        if self.cv_image is None:
            print("F")
            return

        try:
            
            gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        
            # Load the predefined ArUco dictionary (4x4_50 in this case)
            aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

            # Initialize the detector parameters
            aruco_params = cv2.aruco.DetectorParameters()

            # Detect the ArUco markers in the grayscale image
            corners, id, _ = cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=aruco_params)

            if id is not None:
                print("Aruco Marker Detected")
            
            if (id[0] in self.ids): 
                pass
            
            else:
                cv2.imwrite(f"/home/kartikeyan-v10/Inspection/{id[0]}.png", self.cv_image)
                print("Image captured")
                self.ids.append(id[0])
            
            id.clear()
            return
        
        except:
            return

def main():

    rclpy.init()                                       
    node = rclpy.create_node('image_capture')                    
    node.get_logger().info('Node started: image_capture')      
    image = ImageCapture()                                   
    rclpy.spin(image)                                    
    image.destroy_node()                                 
    rclpy.shutdown()                                                

if __name__ == '__main__':

    main()
