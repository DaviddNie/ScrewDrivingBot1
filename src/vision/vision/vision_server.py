import rclpy
import cv2
import tf2_ros
import os
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from std_msgs.msg import String

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PointStamped, Pose, Point, PoseArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


from interfaces.srv import VisionCmd

class VisionServer(Node):

	def __init__(self):
		super().__init__('vision_server')
		# depth camera subscriptions
		self.image_sub = self.create_subscription( Image, '/camera/camera/color/image_raw', self.arm_image_callback, 10)
		self.point_cloud_sub = self.create_subscription( Image, '/camera/camera/aligned_depth_to_color/image_raw', self.arm_point_cloud_callback, 10)
		self.cam_info_sub = self.create_subscription( CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.arm_image_depth_info_callback,10)
		self.intrinsics = None
		self.depth_image = None
		self.vision_status_pub = self.create_publisher(String, 'vision_status', 10)

		# Service definitions
		self.srv = self.create_service(VisionCmd, 'vision_cmd', self.vision_callback)

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


	# This gets bgr image from the image topic
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

	def vision_callback(self, request, response):

		command = request.command

		if (command == "birds_eye"):
			response.pose_array = self.process_birdseye()
		elif (command == "calibrate"):
			response.pose_array = self.process_calibrate()

	def process_calibrate(self):
		self.get_logger().error("to be completed")
			
	def setup_blob_detector_birdseye():
		params = cv2.SimpleBlobDetector_Params()

		# Filter by Area
		params.filterByArea = True
		params.minArea = 150
		params.maxArea = 2000

		# Filter by Circularity
		params.filterByCircularity = True
		params.minCircularity = 0.2

		# Filter by Convexity
		params.filterByConvexity = False

		# Filter by Inertia
		params.filterByInertia = True
		params.minInertiaRatio = 0.8

		# Distance Between Blobs
		params.minDistBetweenBlobs = 10

		return params

	def process_birdseye(self):

		if (self.cv_image is None):
			self.vision_status_pub.publish("Empty Image!")
			return
		
		# Convert image to HSV color space
		hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

		# Setup BlobDetector
		params = self.setup_blob_detector_birdseye()

		# Create a detector with the parameters
		detector = cv2.SimpleBlobDetector_create(params)

		# Copy the image for overlay
		overlay = hsv_image.copy()

		# Detect blobs in the image
		keypoints = detector.detect(hsv_image)

		# Initialize PoseArray
		poseArray = PoseArray()

		# Print Coordinates and prepare to convert them to global
		self.get_logger().debug(f"Detected {len(keypoints)} blobs")

		for i, k in enumerate(keypoints):
			self.get_logger().debug(f"Point {i + 1} is at pixel x = {k.pt[0]/2:.2f}, y = {k.pt[1]/2:.2f}")

			# Convert pixel coordinates to global coordinates
			global_pose = self.pixel_2_global([k.pt[0], k.pt[1]])

			if global_pose is not None:
				# Initialize Pose
				newPose = Pose()

				# Assign global coordinates (adjust with camera offset if necessary)
				newPose.position.x = global_pose[0] - 0.038  # OFFSET TO ACCOUNT FOR CAMERA OFFSET
				newPose.position.y = global_pose[1]
				newPose.position.z = global_pose[2]

				# Add to poseArray
				poseArray.poses.append(newPose)

				# Print global coordinates
				self.get_logger().debug( f"Global coordinates for Point {i + 1}: x = {newPose.position.x}, 
					y = {newPose.position.y}, z = {newPose.position.z}")
			else:
				self.get_logger().warning(f"Could not convert pixel coordinates for Point {i + 1} to global coordinates.")
	
		# Draw detected blobs
		for k in keypoints:
			cv2.circle(overlay, (int(k.pt[0]), int(k.pt[1])), int(k.size/2), (0, 0, 255), -1)
			cv2.line(overlay, (int(k.pt[0])-20, int(k.pt[1])), (int(k.pt[0])+20, int(k.pt[1])), (0,0,0), 3)
			cv2.line(overlay, (int(k.pt[0]), int(k.pt[1])-20), (int(k.pt[0]), int(k.pt[1])+20), (0,0,0), 3)

		# Adjust opacity for the overlay
		opacity = 0.5
		cv2.addWeighted(overlay, opacity, hsv_image, 1 - opacity, 0, hsv_image)

		# Resize the hsv_image to fit the window if needed
		hsv_image = cv2.resize(hsv_image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_CUBIC)

		# Show the result
		# cv2.waitKey(1)
		cv2.imshow("Output", hsv_image)
		cv2.waitKey(1)

		if len(poseArray.poses) == 0:
			self.vision_status_pub.publish(String(data="poseArray is empty!"))
			return

		return poseArray




def main():
	rclpy.init()
	vision_server = VisionServer()
	rclpy.spin(vision_server)
	rclpy.shutdown()


if __name__ == '__main__':
	main()