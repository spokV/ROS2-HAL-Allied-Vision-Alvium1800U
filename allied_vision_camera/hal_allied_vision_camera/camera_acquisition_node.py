#!/usr/bin/env python3
import sys
import os
import shutil
import numpy as np
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
import threading
import rclpy

from typing import List

from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.parameter import Parameter
from rclpy.subscription import Subscription
from ament_index_python.packages import get_package_share_directory


class CameraAcquisition(Node):
    def __init__(self) -> None:
        super().__init__("camera_acquisition")
        self.get_logger().info("Camera acquisition node is awake...")

        self.declare_parameter("acquisition_terminated", "False")

        # Parameters declarations
        self.declare_parameter("number_of_images_to_save", 20)
        self.number_of_images_to_save: int = (
            self.get_parameter("number_of_images_to_save")
            .get_parameter_value()
            .integer_value
        )
        
        self.get_logger().warn(f"self.number_of_images_to_save: {self.number_of_images_to_save}")

        self.declare_parameter("subscribers.camera", "/camera/raw_frame")
        self.camera_topic: str = (
            self.get_parameter("subscribers.camera")
            .get_parameter_value()
            .string_value
        )

        self.declare_parameter("auto_capture.mode", "True")
        self.auto_capture: bool = (
            self.get_parameter("auto_capture.mode").get_parameter_value().bool_value
        )

        self.declare_parameter("auto_capture.time_for_frame", True)
        self.time_for_frame: float = (
            self.get_parameter("auto_capture.time_for_frame")
            .get_parameter_value()
            .double_value
        )

        self.declare_parameter("images_path", "auto")
        self.images_path: str = (
            self.get_parameter("images_path").get_parameter_value().string_value
        )
        

        if self.images_path == "auto":
            package_share_directory = get_package_share_directory(
                "stereo_calibration"
            )
            self.images_path = os.path.join(package_share_directory, "calibration_images/")
        
        self.get_logger().warn(f"Images Path: {self.images_path}")

        self.bridge = CvBridge()
        self.counter_images: int = 0

        self.current_frame: List[float] = []

        self.frame_sub: Subscription = self.create_subscription(
            Image, self.camera_topic, self.callback_frame, 1
        )

        self.acquisition_thread = threading.Thread(
            target=self.acquisition_process, daemon=True
        )
        self.acquisition_thread.start()
    

    def callback_frame(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except CvBridgeError as e:
            print(e)

    def acquisition_process(self):

        while len(self.current_frame) == 0 :
            self.get_logger().warn("Waiting for frame acquisition to start...")
            time.sleep(1)

        if self.auto_capture:
            self.get_logger().info(
                f"\nAuto-Capture - Taking 1 Frame every {self.time_for_frame} secs \n\n"
            )
            self.timer = self.create_timer(self.time_for_frame, self.save_frame)
        else:
            self.get_logger().info(
                "\n############ KEYBOARD COMMANDS ############\n\nq - quit pictures acquisition\nc - capture actual frame\n\n"
            )

        while (
            self.counter_images <= self.number_of_images_to_save
        ):
            rescaled_frame = cv2.resize(self.current_frame, (1000, 1000)) 

            cv2.imshow("LiveCamera -- Camera", rescaled_frame)
            key = cv2.waitKey(50)

            if key == ord("q"):
                self.get_logger().info("Calibration process has been stopped.")
                break
            elif key == ord("c"):
                self.save_frame()

        self.get_logger().info(
            f"Images saved: {self.counter_images}.\n"
        )

        cv2.destroyAllWindows()
        if self.auto_capture:
            self.timer.destroy()

        if (
            self.counter_images >= self.number_of_images_to_save
        ):
            self.get_logger().info(
                "Acquisition Terminated.\n"
            )
            acquisition_param = Parameter(
                "acquisition_terminated", Parameter.Type.BOOL, True
            )
            self.set_parameters([acquisition_param])

        self.destroy_node()

    def save_frame(self):

        img = self.current_frame

        image_name = "image_" + str(self.counter_images) + ".png"
        image_path = os.path.join(self.images_path, image_name)

        writeStatus = cv2.imwrite(
            image_path,
            img
        )

        if writeStatus is True:
            self.get_logger().info("Image {0} written.".format(self.counter_images))
            self.counter_images += 1
        else:
            self.get_logger().info("Error writing image.")


##### Main Function #####
def main(args=None):

    rclpy.init(args=args)
    node = CameraAcquisition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[Camera acquisition node] Node stopped cleanly")
        # node.exit()
    except BaseException:
        node.get_logger().info("[Camera acquisition node] Exception:", file=sys.stderr)
        # node.exit()
        raise
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
