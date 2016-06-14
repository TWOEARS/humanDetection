T1: Launch the camera drivers and broadcast the raw images.
    1. Roscore &
    2. source ~/catkin_ws/devel/setup.bash 
    3. roslaunch ueye_cam stereo.launch     (This step depends on your cameras.)

T2: Launch the stereo_img_proc node. This rectifies raw images.
    1. ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc

T3: Run the human detection module.
    1. humandetection-ros -b
    2. genomixd &
    3. eltclsh
    4. package require genomix
    5. ::genomix::connect
    6. genomix1 load humandetection
    7. Connect the ports accordingly to your cameras setup.
       ::humandetection::connect_port MasterCamera /stereo/left/image_rect_color
       ::humandetection::connect_port SlaveCamera /stereo/right/image_rect_color
       ::humandetection::connect_port LeftCameraParameters /stereo/left/camera_info
       ::humandetection::connect_port RightCameraParameters /stereo/right/camera_info
    8. To start the detection activity (templateMatchingThreshold and disparityThreshold can be adjusted to improve the performance on each system. These are just suggested values. templateMatchingThreshold is in percentage (from 0 to 1) and disparityThreshold is in number of pixels):
       ::humandetection::RunDetection {frameHistory 3 classifiers PATH_TO_CLASSIFIERS/classifiers.txt templateMatchingThreshold 0.75 disparityThreshold 10}
    9. To stop the detection activity just press any key with any of the images active.
    10. To kill the module: 
       ::humandetection::kill
    11. Don't forget to kill genomixd as well: 
       pkill genomixd


note: To have an accurate 3D triangulation, remember to calibrate your system:
    1. rosrun camera_calibration cameracalibrator.py --size 8x5 --square 0.029 right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left
