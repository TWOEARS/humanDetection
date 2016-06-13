T1: Launch the camera drivers and broadcasts the raw images.
    1. Roscore &
    2. source ~/catkin_ws/devel/setup.bash 
    3. roslaunch ueye_cam stereo.launch

T2: Launch the stereo_img_proc node. This rectifies the images and obtaines the disparity image.
    1. ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc

T3: Run the human detection module.
    1. humandetection-ros -b
    2. genomixd &
    3. eltclsh
    4. package require genomix
    5. ::genomix::connect
    6. genomix1 load humandetection
    7. ::humandetection::connect_port MasterCamera /stereo/left/image_rect_color
       ::humandetection::connect_port SlaveCamera /stereo/right/image_rect_color
       ::humandetection::connect_port LeftCameraParameters /stereo/left/camera_info
       ::humandetection::connect_port RightCameraParameters /stereo/right/camera_info
    8. ::humandetection::RunDetection {frameRate 4 temporalSlidingWindowSize 7 detectorFrameRatio 1 maxTrackNumber 100 maxTemplateSize 6 expertThreshold 5 detectionRescalingFactor 0.64 trackRescalingFactor 0.8 outputPath /tmp/humanDetection/ frameHistory 3 classifiers /home/ariel/Documents/LAAS/twoears/repositories/humandetection/classifiers.txt}
    10. ::humandetection::kill
    11. pkill genomixd

T4: (OPTIONAL) Displays Master and Slave rectified images and the Disparity image
    1. rosrun image_view stereo_view stereo:=stereo image:=image_rect_color ap_approximate_sync:=True _queue_size:=10
    1. rosrun image_view disparity_view image:=/stereo/disparity 
T5: (OPTIONAL) To view the pointcloud in rviz
    1. rosrun rviz rviz &
    2. rosrun tf static_transform_publisher 1 1 0 0 0 0 /base_link /camera 500
    3. If there is no visible pointcloud, check that "disparity_range" is ~160.
        rosrun rqt_reconfigure rqt_reconfigure

note: To calibrate:
    1. rosrun camera_calibration cameracalibrator.py --size 8x5 --square 0.029 right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left
