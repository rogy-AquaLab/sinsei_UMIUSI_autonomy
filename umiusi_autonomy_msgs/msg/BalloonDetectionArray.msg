# All balloons the detector reported for one onboard-camera frame.
# header.stamp = the image acquisition time; header.frame_id = the camera optical frame.
std_msgs/Header header
BalloonDetection[] detections
