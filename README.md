# rviz_hrim_plugins


### camera_node

If you need to run a camera node, you can do it with the one that is in this repo in the branch `camera_node`. The node must publish:

 - the image topic using hrim_sensor_camera_msgs::msg::Image
 - the camera_info topic using hrim_sensor_camera_msgs::msg::CameraInfo
 - The transformation. For example a static TF:

```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map image
```
