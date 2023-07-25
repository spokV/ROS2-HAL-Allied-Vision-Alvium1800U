# HAL AlliedVision Alvium Camera

HAL for AlliedVision Alvium Camera ROS2 Package.

## Description

ROS2 Package to interface to the AV Camera.
The package is splitted in core and interfaces.
The package depends on AlliedVision SDK, Vimba 6_1 (https://www.alliedvision.com/en/products/software/#c1500) and the python wrapper, pymba (`git clone https://github.com/morefigs/pymba.git`). 

## Input/Output

Input: 

- rotation_angle: 	angle in degrees for the rotation of the camera feed (double)

Output: 

- raw_frame: 	camera feed (sensor msgs/Image)
	published in camera link

## Calibration

Node to calibrate the camera (OpenCv2).

## Depend

- ROS2 humble
- Vimba 6_1
- opencv with cuda support
