#!/usr/bin/env python3

# Libraries
import rclpy
from rclpy.node import Node
from pymba import *
from sensor_msgs.msg import Image
from allied_vision_camera_interfaces.srv import CameraState
from cv_bridge import CvBridge
import threading
import sys

import numpy as np

from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R

# Class definition of the calibration function
class AVNode(Node):
    def __init__(self):
        super().__init__("av_camera_node")
        self.get_logger().info("AV Camera node is awake...")
        
        # Parameters declaration
        self.declare_parameter("cam_id", 1)

        # Class attributes
        self.cam_id = self.get_parameter("cam_id").value
        self.bridge = CvBridge()
        self.frame = []
        self.start_acquisition = True

        # Acquisition thread
        self.thread1 = threading.Thread(target=self.get_frame, daemon=True)
        self.thread1.start()

        self.declare_parameter("publishers.raw_frame", "/parking_camera/raw_frame")
        self.raw_frame_topic = self.get_parameter("publishers.raw_frame").value

        self.declare_parameter("services.stop_camera", "/parking_camera/stop_camera")
        self.stop_cam_service = self.get_parameter("services.stop_camera").value

        self.declare_parameter("flip_horizontally", "False")
        self.flip_horizontally = self.get_parameter("flip_horizontally").value

        self.declare_parameter("flip_vertically", "False")
        self.flip_vertically = self.get_parameter("flip_vertically").value

        self.declare_parameter("frames.camera_link", "parking_camera_link")
        self.camera_link = self.get_parameter("frames.camera_link").value

        # Publishers
        self.frame_pub = self.create_publisher(Image, self.raw_frame_topic, 1)

        # Service: stop acquisition
        self.stop_service = self.create_service(CameraState, self.stop_cam_service, self.acquisition_service)


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
                self.cam_obj = vimba.camera(0)
                self.cam_obj.open()

                # read a feature value
                feature = self.cam_obj.feature('ExposureAuto')
                feature.value = 'Continuous'

                '''
                # get feature value via feature object
                for feature_name in self.cam_obj.feature_names():
                    feature = self.cam_obj.feature(feature_name)

                    try:
                        value = feature.value
                        range_ = feature.range

                    except VimbaException as e:
                        value = e
                        range_ = None

                    print('\n\t'.join(
                        str(x) for x in (
                            feature_name,
                            'value: {}'.format(value),
                            'range: {}'.format(range_))
                        if x is not None))
                '''

            except:
                self.get_logger().info("No AlliedVision Alvium cameras found, check connection.")
                cam_found = False

            if cam_found:
                try:
                    self.cam_obj.arm("SingleFrame")
                    self.get_logger().info("Frame acquisition has started.")
                    
                    while self.start_acquisition:
                        print("A")
                        current_frame = self.cam_obj.acquire_frame()
                        print("B")
                        self.frame = current_frame.buffer_data_numpy()
                        print("C")
                        self.publish_frame()

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
            print("No Image Returned")
            return
        
        print("D")
        if self.flip_vertically:
            self.frame = np.flip(self.frame, axis=0)

        print("E")
        if self.flip_horizontally:
            self.frame = np.flip(self.frame, axis=1)

        print("QUI")
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
        print('AlliedVision Node stopped cleanly')
        node.exit()
    except BaseException:
        print('Exception in AlliedVision Node:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        rclpy.shutdown() 


# Main
if __name__ == '__main__':
    main()
