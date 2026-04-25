Dump default_cam.yaml into /home/cobot/.ros for USB cam calibration.

If you change any USB cam settings, this file will change and you'll have to run the calibrator:
	ros2 run camera_calibration cameracalibrator \
	  --size 8x6 --square 0.025 \
	  --ros-args \
	  --remap image:=/chessboard/image_raw \
	  --remap camera_info:=/chessboard/camera_info
Click Calibrate (takes a few seconds)
Click Save. writes /tmp/calibrationdata.tar.gz
Extract it:
	cd /tmp && tar xvf calibrationdata.tar.gz
Copy the result:
	mkdir -p ~/.ros/camera_info
	cp /tmp/ost.yaml ~/.ros/camera_info/default_cam.yaml
Also we had the settings:
	v4l2-ctl -d /dev/video0 --set-ctrl=focus_automatic_continuous=0
	v4l2-ctl -d /dev/video0 --set-ctrl=focus_absolute=0
	v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1
	v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=166
If camera ever changes or lighting conditions or whatever, just launch a live video feed and play with the settings until it looks good.
