#!/usr/bin/env python3

# Libraries
import threading
import sys
import numpy as np
import cv2 as cv
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Header

from cv_bridge import CvBridge, CvBridgeError

import vimba
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
        self.resized_dim = 0

        self.thread1 = threading.Thread(target=self.get_frame, daemon=True)
        self.thread1.start()

        self.declare_parameter("publishers.raw_frame", "/camera/raw_frame")
        self.raw_frame_topic = self.get_parameter("publishers.raw_frame").value

        self.declare_parameter("rotation_angle", 0.0)
        self.rotation_angle = float(self.get_parameter("rotation_angle").value)

        self.declare_parameter("frames.camera_link", "camera_link")
        self.camera_link = self.get_parameter("frames.camera_link").value

        self.declare_parameter("resize.enable", False)
        self.need_to_resize_image = self.get_parameter("resize.enable").value

        self.declare_parameter("crop.enable", False)
        self.need_to_crop_image = self.get_parameter("crop.enable").value

        self.declare_parameter("auto_exposure", "Once")
        self.auto_exposure = self.get_parameter("auto_exposure").value

        if self.auto_exposure not in ["Off", "Once", "Continuous"]:
            self.get_logger().info("[AV Camera] Auto Exposure must be Off, Once or Continuous")
            self.auto_exposure = "Once"

        if self.need_to_crop_image:
            self.declare_parameter("crop.cropped_width", 0)
            self.cropped_width = self.get_parameter("crop.cropped_width").value
            self.declare_parameter("crop.cropped_height", 0)
            self.cropped_height = self.get_parameter("crop.cropped_height").value

        if self.need_to_resize_image:
            self.declare_parameter("resize.resized_width", 0)
            self.resized_width = self.get_parameter("resize.resized_width").value
            self.declare_parameter("resize.resized_height", 0)
            self.resized_height = self.get_parameter("resize.resized_height").value
            width = int(self.resized_width)
            height = int(self.resized_height)
            self.resized_dim = (width, height)
            self.get_logger().info("[AV Camera] Resize required with dimensions: {0}".format(self.resized_dim))


        # Publishers
        self.frame_pub = self.create_publisher(Image, self.raw_frame_topic, 1)

        self.get_logger().info("[AV Camera] Node Ready")


    def print_camera(self, cam: vimba.Camera):
        self.get_logger().info('/// Camera Name   : {}'.format(cam.get_name()))
        self.get_logger().info('/// Model Name    : {}'.format(cam.get_model()))
        self.get_logger().info('/// Camera ID     : {}'.format(cam.get_id()))
        self.get_logger().info('/// Serial Number : {}'.format(cam.get_serial()))
        self.get_logger().info('/// Interface ID  : {}\n'.format(cam.get_interface_id()))


    def get_camera(self, camera_id):
        with vimba.Vimba.get_instance() as vimba_obj:

            if camera_id == "auto":
                self.get_logger().info('Access First Camera Available')
                cams = vimba_obj.get_all_cameras()
                if not cams:
                    self.get_logger().info('No Cameras accessible. Abort.')
                    return False

                self.cam = cams[0]

            else:
                self.get_logger().info('Access Camera {0}'.format(camera_id))
                try:
                    self.cam = vimba_obj.get_camera_by_id(camera_id)

                except vimba.VimbaCameraError:
                    self.get_logger().info('Failed to access Camera \'{}\'. Abort.'.format(camera_id))
                    return False

        self.get_logger().info('Camera {0}'.format(self.cam))
        return True

    def setup_camera(self):
        with self.cam:
            # Enable auto exposure time setting if camera supports it
            try:
                #self.cam.ExposureAuto.set('Continuous')
                self.cam.ExposureAuto.set('Off')

            except (AttributeError, VimbaFeatureError):
                pass

            # Enable white balancing if camera supports it
            try:
                #self.cam.BalanceWhiteAuto.set('Continuous')
                self.cam.BalanceWhiteAuto.set('Once')

            except (AttributeError, VimbaFeatureError):
                pass

            #self.cam.AcquisitionFrameRate.set(30)
            self.cam.ExposureTime.set(86000)
            self.cam.ExposureAutoMax.set(86000)
            self.cam.DeviceLinkThroughputLimit.set(400000000)
            self.cam.Gain.set(20)
            # Try to adjust GeV packet size. This Feature is only available for GigE - Cameras.
            #try:
            #    self.cam.GVSPAdjustPacketSize.run()

            #    while not self.cam.GVSPAdjustPacketSize.is_done():
            #        pass

            #except (AttributeError, VimbaFeatureError):
            #    pass

            # Query available, open_cv compatible pixel formats
            # prefer color formats over monochrome formats
            cv_fmts = intersect_pixel_formats(self.cam.get_pixel_formats(), OPENCV_PIXEL_FORMATS)
            color_fmts = intersect_pixel_formats(cv_fmts, COLOR_PIXEL_FORMATS)

            if color_fmts:
                self.cam.set_pixel_format(color_fmts[0])

            else:
                mono_fmts = intersect_pixel_formats(cv_fmts, MONO_PIXEL_FORMATS)

                if mono_fmts:
                    self.cam.set_pixel_format(mono_fmts[0])

                else:
                    abort('Camera does not support a OpenCV compatible format natively. Abort.')

    def _setup_camera(self):
        with self.cam:
            # Try to adjust GeV packet size. This Feature is only available for GigE - Cameras.
            try:
                self.cam.GVSPAdjustPacketSize.run()

                while not self.cam.GVSPAdjustPacketSize.is_done():
                    pass

            except (AttributeError, vimba.VimbaFeatureError):
                pass

    
    def print_feature(self, feature):
        try:
            value = feature.get()

        except (AttributeError, vimba.VimbaFeatureError):
            value = None

        self.get_logger().info('/// Feature name   : {}'.format(feature.get_name()))
        self.get_logger().info('/// Display name   : {}'.format(feature.get_display_name()))
        self.get_logger().info('/// Tooltip        : {}'.format(feature.get_tooltip()))
        self.get_logger().info('/// Description    : {}'.format(feature.get_description()))
        self.get_logger().info('/// SFNC Namespace : {}'.format(feature.get_sfnc_namespace()))
        self.get_logger().info('/// Unit           : {}'.format(feature.get_unit()))
        self.get_logger().info('/// Value          : {}\n'.format(str(value)))

    
    def list_cam_features(self, camera):
        for feature in camera.get_all_features():
            self.print_feature(feature)

    def _frame_handler(self, cam: Camera, frame: Frame):
        if frame.get_id() % 100 == 0:
            print('{} acquired {}'.format(cam, frame), flush=True)

        cam.queue_frame(frame)

    # This function save the current frame in a class attribute
    def get_frame(self):

        with vimba.Vimba.get_instance():
            if self.get_camera(self.camera_name):

                with self.cam:
                    self.setup_camera()

                    self.cam.get_feature_by_name('ExposureAuto').set(self.auto_exposure)

                    #### LIST FEATURES
                    #self.list_cam_features(self.cam)
                    allocation_mode = AllocationMode.AnnounceFrame
                    handler = Handler(self.need_to_resize_image, self.resized_dim, self.get_clock(), self.camera_link, self.frame_pub)

                    self.get_logger().info("[AV Camera] Frame acquisition has started.")
                    """
                    while self.start_acquisition:

                        try:
                            current_frame = self.cam.get_frame(timeout_ms=3000)

                            if current_frame.get_status() == vimba.FrameStatus.Complete:
                                self.frame = current_frame
                                #self.frame = current_frame.as_opencv_image()
                                self.publish_frame()
                            else:
                                self.get_logger().info("Frame Incomplete")

                        except vimba.VimbaTimeout:
                            print("Timeout.. Retry")
                    """
                    if self.start_acquisition:
                        
                        try:
                            #self.cam.start_streaming(handler=self.frame_handler, buffer_count=300, allocation_mode=allocation_mode)
                            self.cam.start_streaming(handler=handler, buffer_count=30)
                            handler.shutdown_event.wait()

                        except vimba.VimbaTimeout:
                            print("Timeout.. Retry")
                            self.cam.stop_streaming()
                        finally:
                            self.cam.stop_streaming()
                    
                self.get_logger().info("Releasing Camera")

            else:
                self.get_logger().info("[AV Camera] No AV Camera Found. Check Connection.")

    def exit(self):
        self.start_acquisition = False
        self.cam.stop_streaming()
        time.sleep(1.0)
        self.thread1.join()


    def publish_frame(self):
    #def frame_handler(self, cam: Camera, frame: Frame):
        #if self.frame.get_id() % 10 == 0:
        print('acquired {}'.format(self.frame), flush=True)
            #self.get_logger().info("[AV Camera] publish {}".format(self.frame.get_id()))
            #print('acquired {}'.format(self.frame.get_id()), flush=True)

        try:

            self.frame = self.frame.as_opencv_image()
            if len(self.frame) == 0:
                self.get_logger().info("[AV Camera] No Image Returned")
                return
            
            if self.need_to_crop_image:
                cur_height, cur_width, _ = current_frame.shape

                delta_width = int((cur_width - self.cropped_width) / 2.0)
                delta_height = int((cur_height - self.cropped_height) / 2.0)
                self.frame = self.frame[delta_height:(cur_height-delta_height), delta_width:(cur_width-delta_width)]
            """ 
            if self.need_to_resize_image:
                self.frame = cv.resize(self.frame, self.resized_dim, interpolation = cv.INTER_AREA)

            if self.rotation_angle != 0.0:
                image_center = tuple(np.array(current_frame.shape[1::-1]) / 2)
                rot_mat = cv.getRotationMatrix2D(image_center, self.rotation_angle, 1.0)
                self.frame = cv.warpAffine(self.frame, rot_mat, current_frame.shape[1::-1], flags=cv.INTER_LINEAR)

            self.image_message = self.bridge.cv2_to_imgmsg(self.frame)
            self.image_message.header = Header()
            self.image_message.header.stamp = self.get_clock().now().to_msg()
            self.image_message.header.frame_id = self.camera_link
            self.frame_pub.publish(self.image_message)
            """
            #cam.queue_frame(frame)        
        except CvBridgeError as e:
            print(e)

class Handler:
    def __init__(self, need_to_resize_image, resized_dim, clock_node, camera_link, frame_pub):
        self.shutdown_event = threading.Event()
        self.resized_dim = resized_dim
        self.need_to_resize_image = need_to_resize_image
        self.bridge = CvBridge()
        self.frame = []
        self.clock_node = clock_node
        self.camera_link = camera_link
        self.frame_pub = frame_pub

    def __call__(self, cam: Camera, frame: Frame):
        ENTER_KEY_CODE = 13

        key = cv.waitKey(1)
        if key == ENTER_KEY_CODE:
            self.shutdown_event.set()
            return

        elif frame.get_status() == FrameStatus.Complete:
            if frame.get_id() % 100 == 0:
                #self.get_logger().info('/// acquired  : {}'.format(frame.get_id()))
                print('acquired {}'.format(frame), flush=True)

            self.frame = frame.as_opencv_image()
            
            if self.need_to_resize_image:
                src = cv.cuda_GpuMat()
                src.upload(self.frame)
                dst = cv.cuda.resize(src, self.resized_dim, interpolation = cv.INTER_CUBIC)
                self.frame = dst.download()

            self.image_message = self.bridge.cv2_to_imgmsg(self.frame)
            self.image_message.header = Header()
            self.image_message.header.stamp = self.clock_node.now().to_msg()
            self.image_message.header.frame_id = self.camera_link
            self.frame_pub.publish(self.image_message)
            
        else:
            print('here')
        cam.queue_frame(frame)

##### Main Function #####
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


if __name__ == "__main__":
    main()
