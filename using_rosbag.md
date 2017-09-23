Udacity provides a ROS BAG file with test data from the real car. Download the file from https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing

You can find more info about the topics that the ROS bag files uses:

$ rosbag info /home/student/loop_with_traffic_light.bag 
path:        /home/student/loop_with_traffic_light.bag
version:     2.0
duration:    1:16s (76s)
start:       Aug 23 2017 14:43:50.31 (1503524630.31)
end:         Aug 23 2017 14:45:06.99 (1503524706.99)
size:        1.6 GB
messages:    1883
compression: none [1152/1152 chunks]
types:       geometry_msgs/PoseStamped [d3812c3cbc69362b77dc0b19b345f8f5]
             sensor_msgs/Image         [060021388200f6f0f447d0fcd9c64743]
topics:      /current_pose    732 msgs    : geometry_msgs/PoseStamped
             /image_raw      1151 msgs    : sensor_msgs/Image

And replay it:

$ rosbag play /home/student/loop_with_traffic_light.bag 
[ INFO] [1505764344.533851997]: Opening /home/student/loop_with_traffic_light.bag

Waiting 0.2 seconds after advertising topics... done.

Hit space to toggle paused, or 's' to step.
 [RUNNING]  Bag Time: 1503524706.910078   Duration: 76.600571 / 76.678469               4.48 
Done.

You should be able to see the current pose in the /current_pose topic as well as the camera image from /image_raw

Then you can use RViz to visualize this data.
