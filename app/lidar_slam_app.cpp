//
// Created by Zhang Zhimeng on 22-10-28.
//
#include "common/file_manager.h"
#include "common/keyframe.h"
#include "slam/system.h"

#include "3rd/backward.hpp"

#include <glog/logging.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

namespace backward
{
backward::SignalHandling sh;
}

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true;

    CHECK(MakeDirs(KeyFrame::kKeyFramePath) == 0)
        << "Failed to create folder: " << KeyFrame::kKeyFramePath;

    rclcpp::init(argc, argv);

    const auto node_handle_ptr = std::make_shared<rclcpp::Node>("funny_lidar_slam_node");

    System system(node_handle_ptr);

    rclcpp::Rate rate(1000);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node_handle_ptr);

        system.Run();

        rate.sleep();
    }

    google::ShutdownGoogleLogging();
    rclcpp::shutdown();

    return 0;
}
