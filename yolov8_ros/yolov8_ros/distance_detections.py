import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from yolov8_msgs.msg import DetectionArray
import cv2
import numpy as np

class ObjectDistanceNode(Node):
    def __init__(self):
        super().__init__('object_distance_node')
        self.create_subscription(CameraInfo, '/multisense/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/multisense/left/depth', self.depth_callback, 10)
        self.create_subscription(DetectionArray, '/yolo_objects/detections', self.detection_callback, 10)

        self.camera_info = None
        self.depth_image = None

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        # Convertir mensaje de ROS a imagen de OpenCV
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_image = depth_image

    def detection_callback(self, msg):
        for detection in msg.detections:
            # Encontrar la caja delimitadora del objeto detectado
            xmin = detection.bounding_box[0].xmin
            ymin = detection.bounding_box[0].ymin
            xmax = detection.bounding_box[0].xmax
            ymax = detection.bounding_box[0].ymax

            # Encontrar el centro de la caja delimitadora
            x_center = int((xmin + xmax) / 2)
            y_center = int((ymin + ymax) / 2)

            # Usar las coordenadas del centro para buscar en la imagen de profundidad
            if self.depth_image is not None:
                distance = self.depth_image[y_center][x_center]
                self.get_logger().info(f'Distance to {detection.class_id}: {distance}')

def main(args=None):
    rclpy.init(args=args)

    object_distance_node = ObjectDistanceNode()

    rclpy.spin(object_distance_node)

    object_distance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
