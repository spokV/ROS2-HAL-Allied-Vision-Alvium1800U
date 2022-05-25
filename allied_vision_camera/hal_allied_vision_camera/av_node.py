#!/usr/bin/env python3

# Libraries
import threading
import sys
import numpy as np
import cv2 as cv

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Header

from cv_bridge import CvBridge

from allied_vision_camera_interfaces.srv import CameraState
from vimba import *


# Class definition of the calibration function
class AVNode(Node):
    def __init__(self):
        super().__init__("av_node")
        
        self.declare_parameter("camera_name", "auto")

        self.camera_name = self.get_parameter("camera_name").value
        self.bridge = CvBridge()
        self.frame = []
        self.start_acquisition = True

        self.thread1 = threading.Thread(target=self.get_frame, daemon=True)
        self.thread1.start()

        self.declare_parameter("publishers.raw_frame", "/camera/raw_frame")
        self.raw_frame_topic = self.get_parameter("publishers.raw_frame").value

        self.declare_parameter("services.stop_camera", "/camera/stop_camera")
        self.stop_cam_service = self.get_parameter("services.stop_camera").value

        self.declare_parameter("rotation_angle", "0.0")
        self.rotation_angle = float(self.get_parameter("rotation_angle").value)

        self.declare_parameter("frames.camera_link", "camera_link")
        self.camera_link = self.get_parameter("frames.camera_link").value


        self.declare_parameter("width", "auto")
        self.width = self.get_parameter("width").value
        self.declare_parameter("height", "auto")
        self.height = self.get_parameter("height").value

        self.declare_parameter("squared_image", "False")
        self.is_squared_image = self.get_parameter("squared_image").value
        

        if self.width != "auto" and self.height != "auto":
            self.resize_image = True
            width = int(self.width)
            height = int(self.height)
            self.resized_dim = (width, height)
            self.get_logger().info("[AV Camera] Resize required with dimensions: {0}".format(self.resized_dim))
        else:
            self.resize_image = False

        # Publishers
        self.frame_pub = self.create_publisher(Image, self.raw_frame_topic, 1)

        # Service: stop acquisition
        self.stop_service = self.create_service(CameraState, self.stop_cam_service, self.acquisition_service)
        self.get_logger().info("[AV Camera] Node Ready")


    def print_camera(self, cam: Camera):
        self.get_logger().info('/// Camera Name   : {}'.format(cam.get_name()))
        self.get_logger().info('/// Model Name    : {}'.format(cam.get_model()))
        self.get_logger().info('/// Camera ID     : {}'.format(cam.get_id()))
        self.get_logger().info('/// Serial Number : {}'.format(cam.get_serial()))
        self.get_logger().info('/// Interface ID  : {}\n'.format(cam.get_interface_id()))


    def acquisition_service(self, request, response):
        self.start_acquisition = request.command_state
        response.cam_state = self.start_acquisition
        return response


    def get_camera(self, camera_id):
        with Vimba.get_instance() as vimba:
            if camera_id == "auto":
                self.get_logger().info('Access First Camera Available')
                cams = vimba.get_all_cameras()
                if not cams:
                    self.get_logger().info('No Cameras accessible. Abort.')
                    return False

                self.cam = cams[0]


            else:
                self.get_logger().info('Access Camera {0}'.format(camera_id))
                try:
                    self.cam = vimba.get_camera_by_id(camera_id)

                except VimbaCameraError:
                    self.get_logger().info('Failed to access Camera \'{}\'. Abort.'.format(camera_id))
                    return False

        return True


    def setup_camera(self):
        with self.cam:
            # Try to adjust GeV packet size. This Feature is only available for GigE - Cameras.
            try:
                self.cam.GVSPAdjustPacketSize.run()

                while not self.cam.GVSPAdjustPacketSize.is_done():
                    pass

            except (AttributeError, VimbaFeatureError):
                pass



    # This function save the current frame in a class attribute
    def get_frame(self):

        with Vimba.get_instance():
            if self.get_camera(self.camera_name):

                with self.cam:
                    self.setup_camera()
                    
                    self.get_logger().info("[AV Camera] Frame acquisition has started.")
                    
                    while self.start_acquisition:
                        for current_frame in self.cam.get_frame_generator(limit=1, timeout_ms=3000):
                            
                            if current_frame.get_status() == FrameStatus.Complete:
                                self.frame = current_frame.as_opencv_image()
                                self.publish_frame()
                            else:
                                self.get_logger().info("Frame Incomplete")
            else:
                self.get_logger().info("[AV Camera] No AV Camera Found. Check Connection.")



    def exit(self):
        self.start_acquisition = False
        self.thread1.join()


    def publish_frame(self):
        
        if len(self.frame) == 0:
            self.get_logger().info("[AV Camera] No Image Returned")
            return

        if self.resize_image:
            height, width, channel = self.frame.shape

            if self.is_squared_image:
                if height <= width:
                    delta_width = int((width - height) / 2.0)
                    squared_frame = self.frame[:, delta_width:width-delta_width]
                else:
                    delta_height = int((height - width) / 2.0)
                    squared_frame = self.frame[delta_height:height-delta_height, :]

                self.frame = cv.resize(squared_frame, self.resized_dim, interpolation = cv.INTER_AREA)
            else:   
                self.frame = cv.resize(self.frame, self.resized_dim, interpolation = cv.INTER_AREA)
        
        if self.rotation_angle != 0.0:
            image_center = tuple(np.array(self.frame.shape[1::-1]) / 2)
            rot_mat = cv.getRotationMatrix2D(image_center, self.rotation_angle, 1.0)
            self.frame = cv.warpAffine(self.frame, rot_mat, self.frame.shape[1::-1], flags=cv.INTER_LINEAR)

        self.image_message = self.bridge.cv2_to_imgmsg(self.frame, encoding="mono8")
        self.image_message.header = Header()
        self.image_message.header.stamp = self.get_clock().now().to_msg()
        self.image_message.header.frame_id = self.camera_link
        self.frame_pub.publish(self.image_message)



# Main loop function
def main(args=None):

    rclpy.init(args=args)
    node = AVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[AV Camera] Node stopped cleanly')
        node.exit()
    except BaseException:
        node.get_logger().info('[AV Camera] Exception:', file=sys.stderr)
        node.exit()
        raise
    finally:
        rclpy.shutdown() 


# Main
if __name__ == '__main__':
    main()
