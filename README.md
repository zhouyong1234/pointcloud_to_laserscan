## pointcloud_to_laserscan
# 将深度摄像头3D点云数据转化为2D激光雷达数据,并与原始2D激光雷达数据相融合
##### pointcloud_to_laserscan_node.launch为启动节点
### 3D点云数据转2D激光雷达数据
##### Subscribed Topics
 - topic: /camera_01/depth/points
 - type: sensor_msgs/PointCloud2
 - topic: /camera_02/depth/points
 - type: sensor_msgs/PointCloud2
##### Published Topics
 - topic: /camera_01/scan
 - type: sensor_msgs/LaserScan
 - topic: /camera_02/scan
 - type: sensor_msgs/LaserScan
### 2D激光数据融合
##### Subscribed Topics
 - topic: /scan
 - type: sensor_msgs/LaserScan
 - topic: /camera_01/scan
 - type: sensor_msgs/LaserScan
 - topic: /camera_02/scan
 - type: sensor_msgs/LaserScan
##### Published Topics
 - topic: /combined_scan
 - type: sensor_msgs/LaserScan
