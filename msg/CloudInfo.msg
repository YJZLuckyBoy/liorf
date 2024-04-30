# Cloud Info
std_msgs/Header header 

int64 imuavailable
int64 odomavailable

# Attitude for LOAM initialization
float32 imurollinit
float32 imupitchinit
float32 imuyawinit

# Initial guess from imu pre-integration
float32 initialguessx
float32 initialguessy
float32 initialguessz
float32 initialguessroll
float32 initialguesspitch
float32 initialguessyaw

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed
sensor_msgs/PointCloud2 cloud_corner    # extracted corner feature
sensor_msgs/PointCloud2 cloud_surface   # extracted surface feature

# 3rd party messages
sensor_msgs/PointCloud2 key_frame_cloud
sensor_msgs/PointCloud2 key_frame_color
sensor_msgs/PointCloud2 key_frame_poses
sensor_msgs/PointCloud2 key_frame_map
