#!/usr/bin/env python3

# Libraries
import rclpy
from rclpy.node import Node
import cv2
from pymba import *
from sensor_msgs.msg import Image
from allied_vision_camera_interfaces.srv import CameraState
from cv_bridge import CvBridge
import threading
import sys

from std_msgs.msg import Header
import tf2_ros
import geometry_msgs
from scipy.spatial.transform import Rotation as R

# Class definition of the calibration function
class AVNode(Node):
	def __init__(self):
		super().__init__("av_camera_node")
		self.get_logger().info("AV Camera node is awake...")
		
		# Parameters declaration
		self.declare_parameter("cam_id", 0)

		# Class attributes
		self.cam_id = self.get_parameter("cam_id").value
		self.cam_obj = None
		self.bridge = CvBridge()
		self.frame = []
		self.start_acquisition = True

		# Acquisition thread
		self.thread1 = threading.Thread(target=self.get_frame, daemon=True)
		self.thread1.start()

		# Publishers
		self.frame_pub = self.create_publisher(Image, "/camera/raw_frame", 2)
		self.timer = self.create_timer(0.03, self.publish_frame)

		# Service: stop acquisition
		self.stop_service = self.create_service(CameraState, "/camera/get_cam_state", self.acquisition_service)


	# This function stops/enable the acquisition stream
	def acquisition_service(self, request, response):
		self.start_acquisition = request.command_state
		response.cam_state = self.start_acquisition
		return response


	# This function save the current frame in a class attribute
	def get_frame(self):
		
		with Vimba() as vimba:
			
			cam_found = True

			try:
				# Open the cam and set the mode
				self.cam_obj = vimba.camera(self.cam_id)
				self.cam_obj.open()

			except:
				self.get_logger().info("No AlliedVision Alvium cameras found, check connection.")
				cam_found = False

			if cam_found:
				try:
					self.cam_obj.arm("SingleFrame")
					self.get_logger().info("Frame acquisition has started.")
					
					while self.start_acquisition:
						current_frame = self.cam_obj.acquire_frame()
						self.frame = current_frame.buffer_data_numpy()

					self.cam_obj.disarm()
					self.cam_obj.close()
				except:
					self.cam_obj.disarm()
					self.cam_obj.close()

				
	# This function stops/enable the acquisition stream
	def exit(self):
		self.start_acquisition = False
		self.thread1.join()



	# Publisher function
	def publish_frame(self):
		
		if len(self.frame) == 0:
			return

		self.image_message = self.bridge.cv2_to_imgmsg(self.frame, encoding="mono8")
		self.image_message.header = Header()
		self.image_message.header.stamp = self.get_clock().now().to_msg()
		self.image_message.header.frame_id = "parking_camera_link"
		self.frame_pub.publish(self.image_message)



# Main loop function
def main(args=None):

	rclpy.init(args=args)
	node = AVNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		print('AlliedVision Node stopped cleanly')
		node.exit()
	except BaseException:
		print('Exception in AlliedVision Node:', file=sys.stderr)
		raise
	finally:
		# Destroy the node explicitly
		# (optional - Done automatically when node is garbage collected)
		node.destroy_node()
		rclpy.shutdown() 


# Main
if __name__ == '__main__':
	main()
