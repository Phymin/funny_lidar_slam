//
// Created by Zhang Zhimeng on 22-5-30.
//

#ifndef FUNNY_LIDAR_SLAM_ROS_UTILITY_H
#define FUNNY_LIDAR_SLAM_ROS_UTILITY_H

#include "common/constant_variable.h"
#include "common/data_type.h"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>

inline Vec3d RosVec3dToEigen(const geometry_msgs::msg::Vector3_<std::allocator<void>>& v)
{
    return {v.x, v.y, v.z};
}

inline Vec3d RosPoint3dToEigen(const geometry_msgs::msg::Point_<std::allocator<void>>& p)
{
    return {p.x, p.y, p.z};
}

inline Eigen::Quaterniond
RosQuaternionToEigen(const geometry_msgs::msg::Quaternion_<std::allocator<void>>& q)
{
    return {q.w, q.x, q.y, q.z};
}

inline uint64_t RosTimeToUs(const std_msgs::msg::Header_<std::allocator<void>>& header)
{
    return header.stamp.sec * 1000000ul + header.stamp.nanosec / 1000ul;
}

// template <typename MessageT>
inline void PublishRosCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                            const PCLPointCloudXYZI::Ptr& cloud)
{
    if (pub->get_subscription_count() == 0)
    {
        return;
    }

    sensor_msgs::msg::PointCloud2 cloud_ros;
    pcl::toROSMsg(*cloud, cloud_ros);
    cloud_ros.header.frame_id = kRosMapFrameID;
    pub->publish(cloud_ros);
}

template <typename T>
inline T getParam(rclcpp::Node::SharedPtr nodeHandle, const std::string& paramName,
                  T defaultVal = T())
{
    try
    {
        if (!nodeHandle->has_parameter(paramName))
        {
            nodeHandle->declare_parameter(paramName, rclcpp::ParameterValue(defaultVal));
        }
    }
    catch (const std::exception& e)
    {
        std::cout << "Catch declare exception: " << e.what() << std::endl;
    }

    return nodeHandle->get_parameter(paramName).get_value<T>();
}

#endif // FUNNY_LIDAR_SLAM_ROS_UTILITY_H
