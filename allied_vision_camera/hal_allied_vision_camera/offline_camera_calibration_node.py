import sys

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from typing import List

from hal_allied_vision_camera.offline_camera_calibration import CameraCalibration


class CameraCalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_calibration")
        self.get_logger().info("Calibration node is awake...")

        self.declare_parameter("acquisition_terminated", "False")

        # Parameters declarations

        self.declare_parameter("image_size", [800, 600])
        self.image_size = (
            self.get_parameter("image_size").get_parameter_value().integer_array_value
        )

        self.declare_parameter("chessboard_size", [6, 8])
        self.chessboard_size = (
            self.get_parameter("chessboard_size")
            .get_parameter_value()
            .integer_array_value
        )

        self.declare_parameter("square_size", 20.0)
        self.square_size = (
            self.get_parameter("square_size").get_parameter_value().double_value
        )

        self.declare_parameter("minimum_valid_images", 20)
        self.minimum_valid_images = (
            self.get_parameter("minimum_valid_images")
            .get_parameter_value()
            .integer_value
        )

        self.declare_parameter("calibration_path", "auto")
        self.calibration_path = (
            self.get_parameter("calibration_path").get_parameter_value().string_value
        )


        self.declare_parameter("display_calibration_feedback", True)
        self.display_calibration_feedback = (
            self.get_parameter("display_calibration_feedback").get_parameter_value().bool_value
        )

        if self.calibration_path == "auto":
            package_share_directory = get_package_share_directory("hal_allied_vision_camera")
            self.calibration_path = package_share_directory + "/calibration/"
        self.get_logger().warn(f"Calibration Path: {self.calibration_path}")


        self.declare_parameter("images_path", "auto")
        self.images_path = (
            self.get_parameter("images_path").get_parameter_value().string_value
        )

        if self.images_path == "auto":
            package_share_directory = get_package_share_directory("hal_allied_vision_camera")
            self.images_path = package_share_directory + "/calibration_images/"
    
        self.get_logger().warn(f"Calibration Images Path: {self.images_path}")

        self.declare_parameter("calibration_file", "calib_params_cam.xml")
        self.calibration_file = (
            self.get_parameter("calibration_file").get_parameter_value().string_value
        )
    
        #################### SINGLE CAMERA CALIBRATION ####################

        cam_calibration = CameraCalibration()

        cam_params = cam_calibration.calibrate(
            self.images_path,
            self.chessboard_size,
            self.square_size,
            self.image_size,
            self.minimum_valid_images,
            display = self.display_calibration_feedback
        )
        self.get_logger().info(
            f'Valid images found: {cam_params["valid_images"]}.'
        )

        if not cam_params:
            raise Exception(
                "Minimum number of valid images not reached in camera calibration."
            )

        try:
            cam_calibration.save_params(
                self.calibration_path, self.calibration_file
            )
            self.get_logger().info(
                f'Camera calibrated. Mean error: {cam_params["mean_error"]:.5f}.'
            )
            self.get_logger().info(
                f'Calibration Results in: {self.calibration_path + self.calibration_file}'
            )
            
        except Exception as e:
            self.get_logger().info(f"Exception: {e}")


def main(args=None):

    rclpy.init(args=args)
    node = CameraCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[Camera calibration node] Node stopped cleanly")
        # node.exit()
    except BaseException:
        node.get_logger().info("[Camera calibration node] Exception:", file=sys.stderr)
        # node.exit()
        raise
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
