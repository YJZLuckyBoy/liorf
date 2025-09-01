#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE 
// <!-- liorf_yjz_lucky_boy -->
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <common_lib.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
// #include <opencv/cv.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

// <!-- liorf_localization_yjz_lucky_boy -->
std::shared_ptr<CommonLib::common_lib> common_lib_;

enum class SensorType { VELODYNE, OUSTER, LIVOX, ROBOSENSE, MULRAN};

class ParamServer : public rclcpp::Node
{
public:
    string history_policy;
    string reliability_policy;

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    int point_filter_num;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    int imuType;
    float imuRate;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // voxel filter paprams
    float mappingSurfLeafSize ;
    float surroundingKeyframeMapLeafSize;
    float loopClosureICPSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer(std::string node_name, const rclcpp::NodeOptions & options) : Node(node_name, options)
    {   
        declare_parameter<string>("history_policy", "history_keep_last");
        get_parameter("history_policy", history_policy);
        declare_parameter<string>("reliability_policy", "reliability_reliable");
        get_parameter("reliability_policy", reliability_policy);

        declare_parameter<string>("pointCloudTopic", "/points_raw");
        get_parameter("pointCloudTopic", pointCloudTopic);
        declare_parameter<string>("imuTopic", "/imu_correct");
        get_parameter("imuTopic", imuTopic);
        declare_parameter<string>("odomTopic", "/odometry/imu");
        get_parameter("odomTopic", odomTopic);
        declare_parameter<string>("gpsTopic", "/odometry/gps");
        get_parameter("gpsTopic", gpsTopic);

        declare_parameter<string>("lidarFrame", "base_link");
        get_parameter("lidarFrame", lidarFrame);
        declare_parameter<string>("baselinkFrame", "base_link");
        get_parameter("baselinkFrame", baselinkFrame);
        declare_parameter<string>("odometryFrame", "odom");
        get_parameter("odometryFrame", odometryFrame);
        declare_parameter<string>("mapFrame", "map");
        get_parameter("mapFrame", mapFrame);

        declare_parameter<bool>("useImuHeadingInitialization", false);
        get_parameter("useImuHeadingInitialization", useImuHeadingInitialization);
        declare_parameter<bool>("useGpsElevation", false);
        get_parameter("useGpsElevation", useGpsElevation);
        declare_parameter<float>("gpsCovThreshold", 2.0f);
        get_parameter("gpsCovThreshold", gpsCovThreshold);
        declare_parameter<float>("poseCovThreshold", 25.0f);
        get_parameter("poseCovThreshold", poseCovThreshold);

        declare_parameter<bool>("savePCD", false);
        get_parameter("savePCD", savePCD);
        declare_parameter<string>("savePCDDirectory", "/Downloads/LOAM/");
        get_parameter("savePCDDirectory", savePCDDirectory);

        std::string sensorStr;
        declare_parameter<string>("sensor", " ");
        get_parameter("sensor", sensorStr);
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else if (sensorStr == "livox")
        {
            sensor = SensorType::LIVOX;
        } else if  (sensorStr == "robosense") {
            sensor = SensorType::ROBOSENSE;
        }
        else if (sensorStr == "mulran")
        {
            sensor = SensorType::MULRAN;
        } 
        else {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox' or 'robosense' or 'mulran'): " << sensorStr);
            rclcpp::shutdown();
        }

        declare_parameter<int>("N_SCAN", 16);
        get_parameter("N_SCAN", N_SCAN);
        declare_parameter<int>("Horizon_SCAN", 1800);
        get_parameter("Horizon_SCAN", Horizon_SCAN);
        declare_parameter<int>("downsampleRate", 1);
        get_parameter("downsampleRate", downsampleRate);
        declare_parameter<int>("point_filter_num", 3);
        get_parameter("point_filter_num", point_filter_num);
        declare_parameter<float>("lidarMinRange", 1.0f);
        get_parameter("lidarMinRange", lidarMinRange);
        declare_parameter<float>("lidarMaxRange", 1000.0f);
        get_parameter("lidarMaxRange", lidarMaxRange);

        declare_parameter<int>("imuType", 0);
        get_parameter("imuType", imuType);
        declare_parameter<float>("imuRate", 500.0f);
        get_parameter("imuRate", imuRate);
        declare_parameter<float>("imuAccNoise", 0.01f);
        get_parameter("imuAccNoise", imuAccNoise);
        declare_parameter<float>("imuGyrNoise", 0.001f);
        get_parameter("imuGyrNoise", imuGyrNoise);
        declare_parameter<float>("imuAccBiasN", 0.0002f);
        get_parameter("imuAccBiasN", imuAccBiasN);
        declare_parameter<float>("imuGyrBiasN", 0.00003f);
        get_parameter("imuGyrBiasN", imuGyrBiasN);
        declare_parameter<float>("imuGravity", 9.80511f);
        get_parameter("imuGravity", imuGravity);
        declare_parameter<float>("imuRPYWeight", 0.01f);
        get_parameter("imuRPYWeight", imuRPYWeight);

        double ida[] = { 1.0,  0.0,  0.0,
                         0.0,  1.0,  0.0,
                         0.0,  0.0,  1.0};
        std::vector < double > id(ida, std::end(ida));
        declare_parameter("extrinsicRot", id);
        get_parameter("extrinsicRot", extRotV);
        declare_parameter("extrinsicRPY", id);
        get_parameter("extrinsicRPY", extRPYV);
        double zea[] = {0.0, 0.0, 0.0};
        std::vector < double > ze(zea, std::end(zea));
        declare_parameter("extrinsicTrans", ze);
        get_parameter("extrinsicTrans", extTransV);

        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY).inverse();

        declare_parameter<float>("mappingSurfLeafSize", 0.2f);
        get_parameter("mappingSurfLeafSize", mappingSurfLeafSize);
        declare_parameter<float>("surroundingKeyframeMapLeafSize", 0.2f);
        get_parameter("surroundingKeyframeMapLeafSize", surroundingKeyframeMapLeafSize);
        declare_parameter<float>("z_tollerance", 1000.0f);
        get_parameter("z_tollerance", z_tollerance);
        declare_parameter<float>("rotation_tollerance", 1000.0f);
        get_parameter("rotation_tollerance", rotation_tollerance);

        declare_parameter<int>("numberOfCores", 2);
        get_parameter("numberOfCores", numberOfCores);
        declare_parameter<double>("mappingProcessInterval", 0.15f);
        get_parameter("mappingProcessInterval", mappingProcessInterval);

        declare_parameter<float>("surroundingkeyframeAddingDistThreshold", 1.0f);
        get_parameter("surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold);
        declare_parameter<float>("surroundingkeyframeAddingAngleThreshold", 0.2f);
        get_parameter("surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold);
        declare_parameter<float>("surroundingKeyframeDensity", 1.0f);
        get_parameter("surroundingKeyframeDensity", surroundingKeyframeDensity);
        declare_parameter<float>("loopClosureICPSurfLeafSize", 0.3f);
        get_parameter("loopClosureICPSurfLeafSize", loopClosureICPSurfLeafSize);
        declare_parameter<float>("surroundingKeyframeSearchRadius", 50.0f);
        get_parameter("surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius);

        declare_parameter<bool>("loopClosureEnableFlag", false);
        get_parameter("loopClosureEnableFlag", loopClosureEnableFlag);
        declare_parameter<float>("loopClosureFrequency", 1.0f);
        get_parameter("loopClosureFrequency", loopClosureFrequency);
        declare_parameter<int>("surroundingKeyframeSize", 50);
        get_parameter("surroundingKeyframeSize", surroundingKeyframeSize);
        declare_parameter<float>("historyKeyframeSearchRadius", 10.0f);
        get_parameter("historyKeyframeSearchRadius", historyKeyframeSearchRadius);
        declare_parameter<float>("historyKeyframeSearchTimeDiff", 30.0f);
        get_parameter("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff);
        declare_parameter<int>("historyKeyframeSearchNum", 25);
        get_parameter("historyKeyframeSearchNum", historyKeyframeSearchNum);
        declare_parameter<float>("historyKeyframeFitnessScore", 0.3f);
        get_parameter("historyKeyframeFitnessScore", historyKeyframeFitnessScore);


       declare_parameter<float>("globalMapVisualizationSearchRadius", 1e3f);
        get_parameter("globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius);
        declare_parameter<float>("globalMapVisualizationPoseDensity", 10.0);
        get_parameter("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity);
        declare_parameter<float>("globalMapVisualizationLeafSize", 1.0f);
        get_parameter("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize);

        usleep(100);
    }

    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in)
    {
        sensor_msgs::msg::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();

        if (imuType) {
            // rotate roll pitch yaw
            Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
            Eigen::Quaterniond q_final = q_from * extQRPY;
            imu_out.orientation.x = q_final.x();
            imu_out.orientation.y = q_final.y();
            imu_out.orientation.z = q_final.z();
            imu_out.orientation.w = q_final.w();

            if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
            {
                RCLCPP_ERROR(get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
                rclcpp::shutdown();
            }
        }

        return imu_out;
    }
};

template<typename T>
sensor_msgs::msg::PointCloud2 publishCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &thisPub, const T& thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);

    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return rclcpp::Time(msg).seconds();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation;
    tf2::fromMsg(thisImuMsg->orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

rclcpp::QoS QosPolicy(const string &history_policy, const string &reliability_policy)
{
    rmw_qos_profile_t qos_profile;
    if (history_policy == "history_keep_last")
        qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    else if (history_policy == "history_keep_all")
        qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_ALL;

    if (reliability_policy == "reliability_reliable")
        qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    else if (reliability_policy == "reliability_best_effort")
        qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    qos_profile.depth = 2000;

    qos_profile.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE;
    qos_profile.deadline = RMW_QOS_DEADLINE_DEFAULT;
    qos_profile.lifespan = RMW_QOS_LIFESPAN_DEFAULT;
    qos_profile.liveliness = rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    qos_profile.liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
    qos_profile.avoid_ros_namespace_conventions = false;

    return rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
}

#endif
