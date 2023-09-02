#!/usr/bin/env python3
import rclpy
from visualization_msgs.msg import Marker
from rclpy.node import Node

class markerNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('marker_node')
        self.marker_pub = self.create_publisher(Marker,"/avocado", 2)
        self.marker = Marker()

        self.marker.header.frame_id = "base_link"
        self.marker.header.stamp = self.get_clock().now().to_msg() 
        self.marker.ns = ""

        # Shape (mesh resource type - 10)
        self.marker.type = Marker.CYLINDER#10
        self.marker.id = 0
        self.marker.action = Marker.ADD#0#Marker.ADD

        # Note: Must set mesh_resource to a valid URL for a model to appear
        #self.marker.mesh_resource = "file://Bodymesh.dae"
        #self.marker.mesh_resource =  "package://yolov8_ros/yolov8_ros/Avocado.glb"
        #self.marker.mesh_use_embedded_materials = True

        # Scale
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0

        # Color
        self.marker.color.r = 0.0
        self.marker.color.g = 99.0
        self.marker.color.b = 255.0
        self.marker.color.a = 1.0

        # Pose
        self.marker.pose.position.x = 3.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        timer_period = 0.1 #1 second
        self.timer=self.create_timer(timer_period,self.timer_callback)

    def timer_callback(self):
        self.marker_pub.publish(self.marker)
def main():
    rclpy.init()
    node = markerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

