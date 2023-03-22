#!/usr/bin/env python3

# Libraries
import sys 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import json
from time import sleep
import threading
from ament_index_python.packages import get_package_share_directory

CALIB_FILE = "calib_params.json"

CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibration_node")
        self.get_logger().info("Calibration node is awake...")

        # Parameters declarations
        self.declare_parameter("number_of_images_to_calibrate", 10)
        self.number_of_images_to_calibrate = int(self.get_parameter("number_of_images_to_calibrate").value)

        self.declare_parameter("board_dim", [6, 8])
        self.board_dim = self.get_parameter("board_dim").value

        self.declare_parameter("square_size", "28.0")
        self.square_size = float(self.get_parameter("square_size").value)

        self.declare_parameter("subscribers.camera", "/camera/raw_frame")
        self.camera_topic = self.get_parameter("subscribers.camera").value

        self.declare_parameter("auto_capture.mode", "True")
        self.auto_capture = self.get_parameter("auto_capture.mode").value

        self.declare_parameter("auto_capture.time_for_frame", "True")
        self.time_for_frame = self.get_parameter("auto_capture.time_for_frame").value

        self.declare_parameter("calibration_path", "auto")
        self.calibration_path = self.get_parameter("calibration_path").value

        package_share_directory = get_package_share_directory('hal_allied_vision_camera')

        if self.calibration_path != "auto":
            self.calib_path = self.calibration_path
        else:
            self.calib_path = package_share_directory + "/calibration/"

        
        self.get_logger().warn("Calibration Path:  {0}".format(self.calib_path))
        
        # Class attributes
        self.bridge = CvBridge()
        self.current_frame = []
        self.calib_params = {"mtx": [], "dist": []}
        self.calib_pics = []

        self.stop_acquisition = False

        self.frame_sub = self.create_subscription(Image, self.camera_topic, self.callback_frame, 1)

        self.thread1 = threading.Thread(target=self.calib_process, daemon=True)
        self.thread1.start()


    def calib_process(self):
        if os.path.exists(self.calib_path+CALIB_FILE):
            self.get_logger().info(CALIB_FILE + " already exists. Overwrite")
            os.remove(self.calib_path+CALIB_FILE)

        self.get_logger().info("Init Calibration Node...")
        sleep(5)
        while len(self.current_frame) == 0:
            self.get_logger().warn("Waiting for the first frame acquisition...")
            sleep(1)

        self.calibrate_cam()


    def callback_frame(self, msg):
        self.current_frame = self.bridge.imgmsg_to_cv2(msg)
        

    def take_calib_pics(self):

        if not self.auto_capture:
            self.get_logger().info("\n======== KEYBOARD COMMANDS ========\n\nq - quit pictures acquisition\nc - capture actual frame\n\n")
        else:
            self.get_logger().info("\nAuto-Capture - Taking 1 Frame every {0} secs \n\n".format(self.time_for_frame))
            self.timer = self.create_timer(self.time_for_frame, self.update_frames) # 50 Hz

        while True:
            cv2.imshow("LiveCamera", self.current_frame)
            key = cv2.waitKey(500)

            if key == ord("q"):
                self.get_logger().info("Calibration process has been stopped.")
                return False
            if key == ord("c"):
                self.update_frames()
            if self.stop_acquisition:
                return True


    def update_frames(self):

        if self.stop_acquisition:
            if self.auto_capture:
                self.timer.destroy()
        else:
            self.calib_pics.append(self.current_frame)
            self.get_logger().info("Picture " + str(len(self.calib_pics)) + " out of " + str(self.number_of_images_to_calibrate))
            if len(self.calib_pics) == self.number_of_images_to_calibrate:
                self.get_logger().info(f"{self.number_of_images_to_calibrate} have been taken successfully.\n")
                cv2.destroyAllWindows()
                self.stop_acquisition = True
                return True


    def calibrate_cam(self):

        is_done = self.take_calib_pics()

        if not is_done:
            self.get_logger().info("Calibration failed: pictures have not been captured correctly.")
            exit(0)

        self.get_logger().info("Calibration process started")

        objp = np.zeros((self.board_dim[0] * self.board_dim[1], 3), np.float32)
        objp[:, :2] = self.square_size * np.mgrid[0:self.board_dim[1], 0:self.board_dim[0]].T.reshape(-1, 2)

        objpoints = [] 
        imgpoints = []

        self.count_images = 0
        for frame in self.calib_pics:

            if len(frame.shape) == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (self.board_dim[1], self.board_dim[0]), None)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)
                imgpoints.append(corners2)

                self.count_images = self.count_images + 1
        
        self.get_logger().info("Number of Valid Images: {0}".format(self.count_images))

        self.get_logger().info("Obtaining Calibration Parameters")

        # Obtain calibration parameters
        ret, self.calib_params["mtx"], self.calib_params["dist"], rvecs, tvecs = cv2.calibrateCamera(objpoints, \
            imgpoints, gray.shape[::-1], None, None)

        # Evaluate the mean error i.e. the calibration reprojection error
        tot_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], self.calib_params["mtx"], self.calib_params["dist"])
            error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            tot_error += error

        self.get_logger().info("mean error: " + str(tot_error/len(objpoints)) + " pixels" + "\n")
        
        file_path = os.path.join(self.calib_path, CALIB_FILE)
        if os.path.exists(file_path):
            os.remove(file_path)

        try:
            cv_file = cv2.FileStorage(file_path, cv2.FILE_STORAGE_WRITE)

            cv_file.write('mtx', self.calib_params["mtx"])
            cv_file.write('dist', self.calib_params["dist"])

            cv_file.release()

        except FileNotFoundError:
            raise FileNotFoundError(f"The {file_path} directory does not exist")


        self.get_logger().info("Calibration has been completed successfully.\nCalibration path: " + self.calib_path + CALIB_FILE)


##### Main Function #####
def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Calibration Node stopped cleanly')
    except BaseException:
        node.get_logger().info('Exception in Calibration Node:', file=sys.stderr)
        raise


if __name__ == "__main__":
    main()
