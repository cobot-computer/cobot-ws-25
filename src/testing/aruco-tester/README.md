Run the USB Cam:
ros2 run usb_cam usb_cam_node_exe   --ros-args   -p video_device:=/dev/video6   -p pixel_format:=mjpeg2rgb   -p image_width:=1600   -p image_height:=896   -p framerate:=30.0   -p brightness:=-64   -p contrast:=14 -p sharpness:=7



Aruco Transforms (marker detection) debugging output:
ros2 run aruco_transforms aruco_transforms


See Aruco Marker Detection In Live Feed:
-Terminal 1: python3 /home/cobot/cobot-ws-25/cobot-ws-25/src/testing/aruco-tester/aruco_debug.py
-Terminal 2: ros2 run image_view image_view --ros-args -r image:=/aruco_debug/image_raw


View Ouput Of Aruco's Warp:
ros2 run image_view image_view --ros-args -r image:=/chessboard/image_raw
