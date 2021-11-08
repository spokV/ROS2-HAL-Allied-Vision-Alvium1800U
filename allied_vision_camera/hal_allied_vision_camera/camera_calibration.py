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
import time
from time import sleep
import threading
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('hal_allied_vision_camera')
# Path to store the calibration file
CALIB_PATH = package_share_directory + "/calibration/"
CALIB_FILE = "calib_params.json"

CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
SQUARE_SIZE = 28.0 # mm


# Class definition of the calibration function
class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibration_node")
        self.get_logger().info("Calibration node is awake...")

        # Parameters declarations
        self.declare_parameter("number_of_images_to_calibrate", 10)
        self.number_of_images_to_calibrate = int(self.get_parameter("number_of_images_to_calibrate").value)

        self.declare_parameter("board_dim", [6, 8])
        self.board_dim = self.get_parameter("board_dim").value

        self.declare_parameter("subscribers.camera", "/camera/raw_frame")
        self.camera_topic = self.get_parameter("subscribers.camera").value

        self.declare_parameter("auto_capture.mode", "True")
        self.auto_capture = self.get_parameter("auto_capture.mode").value

        self.declare_parameter("auto_capture.time_for_frame", "True")
        self.time_for_frame = self.get_parameter("auto_capture.time_for_frame").value

        self.declare_parameter("calibration_path", "auto")
        self.calibration_path = self.get_parameter("calibration_path").value

        if self.calibration_path != "auto":
            CALIB_PATH = self.calibration_path
        
        # Class attributes
        self.bridge = CvBridge()
        self.current_frame = []
        self.calib_params = {"mtx": [], "dist": []}
        self.calib_pics = []

        self.stop_acquisition = False

        # Subscription from camera stream
        self.frame_sub = self.create_subscription(Image, self.camera_topic, self.callback_frame, 1)

        # Start calibration
        self.thread1 = threading.Thread(target=self.calib_process, daemon=True)
        self.thread1.start()



    # Calibration process
    def calib_process(self):
        # Check if the calibration file already exists and ask for overwriting
        if os.path.exists(CALIB_PATH+CALIB_FILE):
            self.get_logger().info(CALIB_FILE + " already exists. Overwrite")
            os.remove(CALIB_PATH+CALIB_FILE)

            while len(self.current_frame) == 0:
                self.get_logger().warn("Waiting for the first frame acquisition...")
                sleep(1)

            self.calibrate_cam()
        else:
            while len(self.current_frame) == 0:
                self.get_logger().warn("Waiting for the first frame acquisition...")
                sleep(1)

            self.calibrate_cam()


    # This function is a callback for the frame acqusiition
    def callback_frame(self, msg):

        # Save the current frame in a class attribute
        self.current_frame = self.bridge.imgmsg_to_cv2(msg)
        


    # This function let the user save 15 pics of a checkerboard
    def take_calib_pics(self):

        if not self.auto_capture:
            self.get_logger().info("\n======== KEYBOARD COMMANDS ========\n\nq - quit pictures acquisition\nc - capture actual frame\n\n")
        else:
            self.get_logger().info("\nAuto-Capture - Taking 1 Frame each {0} sec \n\n".format(self.time_for_frame))
            self.timer = self.create_timer(self.time_for_frame, self.update_frames) # 50 Hz

        
        while True:
            cv2.imshow("LiveCamera", self.current_frame)
            key = cv2.waitKey(1)

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

    # This functions drives the user to the camera calibration process
    def calibrate_cam(self):

        # At first, ask the user to take 15 pictures
        is_done = self.take_calib_pics()

        if not is_done:
            self.get_logger().info("Calibration failed: pictures have not been captured correctly.")
            exit(0)

        self.get_logger().info("Calibration process started")

        # Calibration process using the taken pictures.
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((self.board_dim[0] * self.board_dim[1], 3), np.float32)
        objp[:, :2] = SQUARE_SIZE * np.mgrid[0:self.board_dim[1], 0:self.board_dim[0]].T.reshape(-1, 2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

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
        
        self.get_logger().info("count_images: {0}".format(self.count_images))

        self.get_logger().info("Obtaining calibration parameters")

        # Obtain calibration parameters
        ret, self.calib_params["mtx"], self.calib_params["dist"], rvecs, tvecs = cv2.calibrateCamera(objpoints, \
            imgpoints, gray.shape[::-1], None, None)

        # Evaluate the mean error i.e. the calibration reprojection error
        tot_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], self.calib_params["mtx"], self.calib_params["dist"])
            error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            tot_error += error

        self.get_logger().info("mean error: " + str(tot_error/len(objpoints)) + "pixels" + "\n")

        # Adjust parameters to be serializable for JSON 
        self.calib_params["mtx"] = [self.calib_params["mtx"].flatten()[i] for i in range(9)]
        self.calib_params["dist"] = [self.calib_params["dist"].flatten()[i] for i in range(5)]

        # Write on the calibration params file	
        with open(CALIB_PATH+CALIB_FILE, "w") as outfile:
            json.dump(self.calib_params, outfile)

        self.get_logger().info("Calibration has been completed successfully.\nCalibration path: " + CALIB_PATH + CALIB_FILE)
        


# Main loop function
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
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        rclpy.shutdown() 


# Main
if __name__ == '__main__':
    main()