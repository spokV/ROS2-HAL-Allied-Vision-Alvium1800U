# HAL AlliedVision Alvium Camera

HAL for AlliedVision Alvium Camera ROS2 Package.

## Description

ROS2 Package to interface to the AV Camera.
The package is splitted in core and interfaces.

## Input/Output

Input: 

- rotation_angle: 	angle in degrees for the rotation of the camera feed (double)

Output: 

- raw_frame: 	camera feed (sensor msgs/Image)
	published in camera link

## Calibration

Node to calibrate the camera (Opencv2)

## Depend

- ROS2
- Vimba 5_0
- pymba
- opencv-python
