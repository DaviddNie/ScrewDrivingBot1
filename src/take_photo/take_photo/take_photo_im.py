import rclpy
import cv2
import tf2_ros
import os
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PointStamped, Pose, Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class take_photo(Node):

	def __init__(self):
		super().__init__('take_photo')
		# depth camera subscriptions
		self.image_sub = self.create_subscription( Image, '/camera/camera/color/image_raw', self.arm_image_callback, 10)
		self.point_cloud_sub = self.create_subscription( Image, '/camera/camera/aligned_depth_to_color/image_raw', self.arm_point_cloud_callback, 10)
		self.cam_info_sub = self.create_subscription( CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.arm_image_depth_info_callback,10)
		self.intrinsics = None
		self.depth_image = None

		# Timer definitions
		self.routine_timer = self.create_timer(0.05, self.routine_callback)

		# Transformation Interface
		self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)

		# General Variables
		self.cv_image = None
		self.mask = None
		self.cv_bridge = CvBridge()



	def arm_image_depth_info_callback(self, cameraInfo):
		try:
			if self.intrinsics:
				return
			self.intrinsics = rs.intrinsics()
			self.intrinsics.width = cameraInfo.width
			self.intrinsics.height = cameraInfo.height
			self.intrinsics.ppx = cameraInfo.k[2]
			self.intrinsics.ppy = cameraInfo.k[5]
			self.intrinsics.fx = cameraInfo.k[0]
			self.intrinsics.fy = cameraInfo.k[4]
			if cameraInfo.distortion_model == 'plumb_bob':
				self.intrinsics.model = rs.distortion.brown_conrady
			elif cameraInfo.distortion_model == 'equidistant':
				self.intrinsics.model = rs.distortion.kannala_brandt4
			self.intrinsics.coeffs = [i for i in cameraInfo.d]
		except CvBridgeError as e:
			print(e)
			return


	# This gets bgr image from the image topic and finds where green in the image is
	def arm_image_callback(self, msg):      
		try:
			self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
			
		except Exception as e:
			self.get_logger().error(f"Error in arm_image_callback: {str(e)}")

		

	# This gets depth_frame aligned with RGB image
	def arm_point_cloud_callback(self, msg):
		try:
			self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
				
		except Exception as e:
			self.get_logger().error(f"Error in point_cloud_callback: {str(e)}")


	# Note that the order of length and width is switched from the original code
	# i.e. the original codebase has pixel_pt[0],pixel_pt[1] instead of pixel_pt[1],pixel_pt[0]
	def pixel_2_global(self, pixel_pt):
		if self.depth_image is not None and self.intrinsics is not None:
			height, width = self.depth_image.shape[:2]
			self.get_logger().info(f"Pixel coordinates {pixel_pt} are in bounds for the depth image of size ({height}, {width})")
			
			
			[x,y,z] = rs.rs2_deproject_pixel_to_point(self.intrinsics, (pixel_pt[1],pixel_pt[0] ), self.depth_image[pixel_pt[1],pixel_pt[0] ]*0.001)
			return [x, y,z]
		else:
			return None

	def routine_callback(self):

		if (self.cv_image is None):
			return
		

		# TODO: COLOUR MASK TO FIND THE CENTER OF AN OBJECT OF INTEREST
		
		# Convert image to HSV color space
		hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

		# Code for displaying the mask
		#
		cv2.waitKey(1)
		cv2.imshow("bla", hsv_image)
		cv2.waitKey(1)


def main():
	rclpy.init()
	object_detect = take_photo()
	rclpy.spin(object_detect)
	rclpy.shutdown()


if __name__ == '__main__':
	main()