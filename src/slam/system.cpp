//
// Created by Zhang Zhimeng on 22-10-27.
//
#include "slam/system.h"
#include "common/constant_variable.h"
#include "common/math_function.h"
#include "common/pointcloud_utility.h"
#include "common/ros_utility.h"
#include "common/sensor_data_utility.h"
#include "lidar/lidar_point_type.h"
#include "optimization/g2o/g2o_optimizer_header.h"
#include "slam/config_parameters.h"
#include "slam/frontend.h"
#include "slam/localization.h"
#include "slam/loop_closure.h"
#include "slam/preprocessing.h"
#include "slam/split_map.h"

#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <fstream>
#include <memory>
#include <utility>

System::System(rclcpp::Node::SharedPtr nodeHandle) : node_handle_ptr_(std::move(nodeHandle))
{
    InitConfigParameters();
    InitLidarModel();
    InitPublisher();
    InitSubscriber();

    imu_data_searcher_ptr_ = std::make_shared<IMUDataSearcher>(
        ConfigParameters::Instance().imu_data_searcher_buffer_size_);

    if (slam_mode_ == SLAM_MODE::MAPPING)
    {
        InitMappingMode();
        LOG(INFO) << "\033[1;32m----> System Started. Mapping Mode \033[0m";
    }
    else if (slam_mode_ == SLAM_MODE::LOCALIZATION)
    {
        LOG(INFO) << "\033[1;32m----> System Started. Localization Mode\033[0m";
        InitLocalizationMode();
    }
    else
    {
        LOG(FATAL) << "Please select SLAM mode";
    }

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle_ptr_);

    pre_processing_ptr_ = new PreProcessing(this);
    pre_processing_thread_ptr_ = new std::thread(&PreProcessing::Run, pre_processing_ptr_);
}

void System::InitMappingMode()
{
    InitSaveMapServer();
    InitMappingPublisher();

    loop_closure_optimizer_ptr_ = std::make_shared<LoopClosureOptimizer>();

    front_end_ptr_ = new FrontEnd(this);
    frontend_thread_ptr_ = new std::thread(&FrontEnd::Run, front_end_ptr_);

    if (ConfigParameters::Instance().system_enable_loopclosure_)
    {
        loop_closure_ptr_ = new LoopClosure(this);
        loop_closure_thread_ptr_ = new std::thread(&LoopClosure::Run, loop_closure_ptr_);
    }

    if (ConfigParameters::Instance().system_enable_visualize_global_map_)
    {
        visualize_global_map_thread_ptr_ = new std::thread(&System::VisualizeGlobalMap, this);
    }
}

void System::InitLocalizationMode()
{
    InitLocalizationPublisher();

    localization_ptr_ = new Localization(this);
    localization_thread_ptr_ = new std::thread(&Localization::Run, localization_ptr_);
}

System::~System()
{
    if (slam_mode_ == SLAM_MODE::MAPPING)
    {
        cv_frontend_.notify_one();
        frontend_thread_ptr_->join();
        delete front_end_ptr_;
        delete frontend_thread_ptr_;

        if (ConfigParameters::Instance().system_enable_loopclosure_)
        {
            loop_closure_thread_ptr_->join();
            delete loop_closure_ptr_;
            delete loop_closure_thread_ptr_;
        }

        if (ConfigParameters::Instance().system_enable_visualize_global_map_)
        {
            visualize_global_map_thread_ptr_->join();
            delete visualize_global_map_thread_ptr_;
        }
    }
    else if (slam_mode_ == SLAM_MODE::LOCALIZATION)
    {
        cv_localization_.notify_one();
        localization_thread_ptr_->join();
        delete localization_ptr_;
        delete localization_thread_ptr_;
    }

    cv_preprocessing_.notify_one();
    pre_processing_thread_ptr_->join();
    delete pre_processing_ptr_;
    delete pre_processing_thread_ptr_;
}

void System::InitLidarModel()
{
    if (ConfigParameters::Instance().lidar_sensor_type_ == "None")
    {
        LidarModel::Instance(ConfigParameters::Instance().lidar_sensor_type_);
        int lidar_horizon_scan = ConfigParameters::Instance().lidar_horizon_scan_;
        LidarModel::Instance()->horizon_scan_num_ = lidar_horizon_scan;
        LidarModel::Instance()->vertical_scan_num_ = ConfigParameters::Instance().lidar_scan_;
        LidarModel::Instance()->h_res_ =
            Degree2Radian(360.0f / static_cast<float>(lidar_horizon_scan));
        LidarModel::Instance()->v_res_ = static_cast<float>(
            Degree2Radian(ConfigParameters::Instance().lidar_vertical_resolution_));
        LidarModel::Instance()->lower_angle_ =
            static_cast<float>(Degree2Radian(ConfigParameters::Instance().lidar_lower_angle_));
    }
    else
    {
        LidarModel::Instance(ConfigParameters::Instance().lidar_sensor_type_);
    }
}

void System::InitConfigParameters()
{
    ConfigParameters& config = ConfigParameters::Instance();

    // sensor topic name
    // CHECK(node_handle_ptr_->getParam("sensor_topic/lidar_topic", config.lidar_topic_));
    // CHECK(node_handle_ptr_->getParam("sensor_topic/imu_topic", config.imu_topic_));

    config.lidar_topic_ = getParam<std::string>(node_handle_ptr_, "sensor_topic.lidar_topic");
    CHECK(!config.lidar_topic_.empty());

    config.imu_topic_ = getParam(node_handle_ptr_, "sensor_topic.imu_topic", config.imu_topic_);
    CHECK(!config.imu_topic_.empty());

    int temp = getParam(node_handle_ptr_, "slam_mode", 0);
    // getParam(node_handle_ptr_, "slam_mode", temp, 0);
    slam_mode_ = static_cast<SLAM_MODE>(temp);

    // lidar config parameters
    config.lidar_sensor_type_ = getParam(node_handle_ptr_, "lidar.lidar_sensor_type", StringEmpty);
    config.lidar_point_jump_span_ =
        getParam(node_handle_ptr_, "lidar.lidar_point_jump_span", IntNaN);
    config.lidar_scan_ = getParam(node_handle_ptr_, "lidar.lidar_scan", IntNaN);
    config.lidar_lower_angle_ = getParam(node_handle_ptr_, "lidar.lidar_lower_angle", DoubleNaN);
    config.lidar_horizon_scan_ = getParam(node_handle_ptr_, "lidar.lidar_horizon_scan", IntNaN);
    config.lidar_vertical_resolution_ =
        getParam(node_handle_ptr_, "lidar.lidar_vertical_resolution", DoubleNaN);
    config.lidar_use_min_dist_ =
        getParam(node_handle_ptr_, "lidar.lidar_use_min_distance", FloatNaN);
    config.lidar_use_max_dist_ =
        getParam(node_handle_ptr_, "lidar.lidar_use_max_distance", FloatNaN);
    config.lidar_point_time_scale_ =
        getParam(node_handle_ptr_, "lidar.lidar_point_time_scale", DoubleNaN);
    config.lidar_rotation_noise_std_ =
        getParam(node_handle_ptr_, "lidar.lidar_rotation_noise_std", DoubleNaN);
    config.lidar_position_noise_std_ =
        getParam(node_handle_ptr_, "lidar.lidar_position_noise_std", DoubleNaN);

    // imu config parameters
    config.imu_has_orientation_ = getParam(node_handle_ptr_, "imu.has_orientation", false);
    config.imu_init_acc_bias_ = getParam(node_handle_ptr_, "imu.init_acc_bias", DoubleNaN);
    config.imu_init_gyro_bias_ = getParam(node_handle_ptr_, "imu.init_gyro_bias", DoubleNaN);
    config.imu_acc_noise_std_ = getParam(node_handle_ptr_, "imu.acc_noise_std", DoubleNaN);
    config.imu_gyro_noise_std_ = getParam(node_handle_ptr_, "imu.gyro_noise_std", DoubleNaN);
    config.imu_acc_rw_noise_std_ = getParam(node_handle_ptr_, "imu.acc_rw_noise_std", DoubleNaN);
    config.imu_gyro_rw_noise_std_ = getParam(node_handle_ptr_, "imu.gyro_rw_noise_std", DoubleNaN);
    config.imu_data_searcher_buffer_size_ =
        getParam(node_handle_ptr_, "imu.data_searcher_buffer_size", IntNaN);

    // gravity
    config.gravity_norm = getParam(node_handle_ptr_, "gravity", DoubleNaN);

    // calibration parameters
    std::vector<double> lidar_to_imu =
        getParam<std::vector<double>>(node_handle_ptr_, "calibration.lidar_to_imu");
    // node_handle_ptr_->getParam("calibration/lidar_to_imu", lidar_to_imu);
    config.calibration_lidar_to_imu_ =
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(lidar_to_imu.data());

    // frontend config parameters
    config.registration_and_searcher_mode_ =
        getParam(node_handle_ptr_, "frontend.registration_and_searcher_mode", StringEmpty);
    config.loam_feature_corner_thres_ =
        getParam(node_handle_ptr_, "frontend.feature.corner_thres", FloatNaN);
    config.loam_feature_planar_thres_ =
        getParam(node_handle_ptr_, "frontend.feature.planar_thres", FloatNaN);
    config.loam_feature_planar_voxel_filter_size_ =
        getParam(node_handle_ptr_, "frontend.feature.planar_voxel_filter_size", FloatNaN);
    config.loam_feature_corner_voxel_filter_size_ =
        getParam(node_handle_ptr_, "frontend.feature.corner_voxel_filter_size", FloatNaN);
    config.registration_line_ratio_thres_ =
        getParam(node_handle_ptr_, "frontend.registration.line_ratio_thres", FloatNaN);
    config.registration_point_search_thres_ =
        getParam(node_handle_ptr_, "frontend.registration.point_search_thres", FloatNaN);
    config.registration_point_to_planar_thres_ =
        getParam(node_handle_ptr_, "frontend.registration.point_to_planar_thres", FloatNaN);
    config.registration_local_planar_map_size_ =
        getParam(node_handle_ptr_, "frontend.registration.local_planar_map_size", IntNaN);
    config.registration_local_corner_map_size_ =
        getParam(node_handle_ptr_, "frontend.registration.local_corner_map_size", IntNaN);
    config.registration_keyframe_delta_dist_ =
        getParam(node_handle_ptr_, "frontend.registration.keyframe_delta_distance", DoubleNaN);
    config.registration_keyframe_delta_rotation_ =
        getParam(node_handle_ptr_, "frontend.registration.keyframe_delta_rotation", DoubleNaN);
    config.registration_rotation_converge_thres_ =
        getParam(node_handle_ptr_, "frontend.registration.rotation_converge_thres", FloatNaN);
    config.registration_position_converge_thres_ =
        getParam(node_handle_ptr_, "frontend.registration.position_converge_thres", FloatNaN);
    config.registration_local_corner_filter_size_ = getParam(
        node_handle_ptr_, "frontend.registration.local_corner_voxel_filter_size", FloatNaN);
    config.registration_local_planar_filter_size_ = getParam(
        node_handle_ptr_, "frontend.registration.local_planar_voxel_filter_size", FloatNaN);
    config.registration_local_map_size_ =
        getParam(node_handle_ptr_, "frontend.registration.local_map_size", IntNaN);
    config.registration_local_map_cloud_filter_size_ =
        getParam(node_handle_ptr_, "frontend.registration.local_map_cloud_filter_size", FloatNaN);
    config.registration_source_cloud_filter_size_ =
        getParam(node_handle_ptr_, "frontend.registration.source_cloud_filter_size", FloatNaN);
    config.registration_opti_iter_num_ =
        getParam(node_handle_ptr_, "frontend.registration.optimization_iter_num", IntNaN);
    config.registration_ndt_voxel_size_ =
        getParam(node_handle_ptr_, "frontend.registration.ndt_voxel_size", DoubleNaN);
    config.registration_ndt_outlier_threshold_ =
        getParam(node_handle_ptr_, "frontend.registration.ndt_outlier_threshold", DoubleNaN);
    config.registration_ndt_min_points_in_voxel_ =
        getParam(node_handle_ptr_, "frontend.registration.ndt_min_points_in_voxel", IntNaN);
    config.registration_ndt_max_points_in_voxel_ =
        getParam(node_handle_ptr_, "frontend.registration.ndt_max_points_in_voxel", IntNaN);
    config.registration_ndt_min_effective_pts_ =
        getParam(node_handle_ptr_, "frontend.registration.ndt_min_effective_pts", IntNaN);
    config.registration_ndt_capacity_ =
        getParam(node_handle_ptr_, "frontend.registration.ndt_capacity", IntNaN);
    config.fusion_method_ = getParam(node_handle_ptr_, "frontend.fusion_method", StringEmpty);
    config.frontend_fusion_opti_iters_ =
        getParam(node_handle_ptr_, "frontend.fusion_opti_iters", IntNaN);

    // system config parameters
    config.system_keyframe_delta_dist_ =
        getParam(node_handle_ptr_, "system.keyframe_delta_distance", DoubleNaN);
    config.system_keyframe_delta_rotation_ =
        getParam(node_handle_ptr_, "system.keyframe_delta_rotation", DoubleNaN);
    config.system_enable_loopclosure_ =
        getParam(node_handle_ptr_, "system.enable_loopclosure", false);
    config.system_enable_visualize_global_map_ =
        getParam(node_handle_ptr_, "system.enable_visualize_global_map", false);
    config.system_global_map_visualization_resolution_ =
        getParam(node_handle_ptr_, "system.global_map_visualization_resolution", FloatNaN);
    // split map
    config.tile_map_grid_size_ = getParam(node_handle_ptr_, "system.tile_map_grid_size", DoubleNaN);

    // loopclosure config parameters
    config.lc_skip_near_loopclosure_threshold_ =
        getParam(node_handle_ptr_, "loopclosure.skip_near_loopclosure_threshold", IntNaN);
    config.lc_skip_near_keyframe_threshold_ =
        getParam(node_handle_ptr_, "loopclosure.skip_near_keyframe_threshold", IntNaN);
    config.lc_candidate_local_map_left_range_ =
        getParam(node_handle_ptr_, "loopclosure.candidate_local_map_left_range", IntNaN);
    config.lc_candidate_local_map_right_range_ =
        getParam(node_handle_ptr_, "loopclosure.candidate_local_map_right_range", IntNaN);
    config.lc_loopclosure_local_map_left_range_ =
        getParam(node_handle_ptr_, "loopclosure.loopclosure_local_map_left_range", IntNaN);
    config.lc_near_neighbor_distance_threshold_ =
        getParam(node_handle_ptr_, "loopclosure.near_neighbor_distance_threshold", DoubleNaN);
    config.lc_registration_converge_threshold_ =
        getParam(node_handle_ptr_, "loopclosure.registration_converge_threshold", FloatNaN);

    LOG(INFO) << "Configure parameters:\n " << config.toString();
}

void System::InitLocalizationPublisher()
{
    localization_path_pub_ =
        node_handle_ptr_->create_publisher<nav_msgs::msg::Path>("/localization_path", 5);
    localization_local_cloud_map_pub_ =
        node_handle_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>("/localization_local_map",
                                                                          5);
    localization_global_cloud_map_pub_ =
        node_handle_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/localization_global_map", 5);
    localization_current_lidar_cloud_pub_ =
        node_handle_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/localization_current_lidar", 5);
}

void System::InitMappingPublisher()
{
    mapping_keyframe_path_cloud_pub_ =
        node_handle_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>("/keyframe_cloud_path",
                                                                          5);
    mapping_keyframe_path_pub_ =
        node_handle_ptr_->create_publisher<nav_msgs::msg::Path>("/keyframe_path", 5);
    mapping_curr_corner_cloud_pub_ =
        node_handle_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>("/corner_cloud", 5);
    mapping_curr_planer_cloud_pub_ =
        node_handle_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>("/planer_cloud", 5);
    mapping_curr_keyframe_cloud_pub_ =
        node_handle_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>("/keyframe_cloud", 2);
    mapping_global_map_cloud_pub_ =
        node_handle_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map_cloud", 2);
}

void System::InitPublisher()
{
    mapping_curr_cloud_pub_ =
        node_handle_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>("/frame_cloud", 100);
}

void System::InitSubscriber()
{
    if (LidarModel::Instance()->lidar_sensor_type_ == LidarModel::LidarSensorType::LIVOX_AVIA)
    {
        // lidar_sub_ = node_handle_ptr_->subscribe(ConfigParameters::Instance().lidar_topic_,
        // 100000,
        //                                          &System::LidarLivoxMsgCallBack, this);
    }
    else
    {
        lidar_sub_ = node_handle_ptr_->create_subscription<sensor_msgs::msg::PointCloud2>(
            ConfigParameters::Instance().lidar_topic_, 100000,
            std::bind(&System::LidarStandardMsgCallBack, this, std::placeholders::_1));
    }

    imu_sub_ = node_handle_ptr_->create_subscription<sensor_msgs::msg::Imu>(
        ConfigParameters::Instance().imu_topic_, 1000000,
        std::bind(&System::ImuMsgCallBack, this, std::placeholders::_1));

    if (slam_mode_ == SLAM_MODE::LOCALIZATION)
    {
        localization_init_pose_sub_ =
            node_handle_ptr_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/initialpose", 1,
                std::bind(&System::LocalizationInitPoseMsgCallBack, this, std::placeholders::_1));
    }
}

void System::InitSaveMapServer()
{
    save_map_server_ = node_handle_ptr_->create_service<funny_lidar_slam::srv::SaveMap>(
        "/funny_lidar_slam/save_map",
        std::bind(&System::SaveMap, this, std::placeholders::_1, std::placeholders::_2));
}

bool System::SaveMap(const std::shared_ptr<funny_lidar_slam::srv::SaveMap::Request> req,
                     std::shared_ptr<funny_lidar_slam::srv::SaveMap::Response> res)
{
    std::lock_guard<std::mutex> lg(mutex_keyframes_);

    res->map_path = StringEmpty;
    res->map_size = 0u;
    res->save_map_succeed = false;

    if (keyframes_.empty())
    {
        return false;
    }

    try
    {

        PCLPointCloudXYZI::Ptr map_cloud(new PCLPointCloudXYZI);
        for (const auto& keyframe : keyframes_)
        {
            auto cloud = VoxelGridCloud(*keyframe->LoadOrderedCloud(), 0.3f);
            *map_cloud += TransformPointCloud(*cloud, keyframe->pose_);
        }

        map_cloud = VoxelGridCloud(*map_cloud, 0.3);

        if (!map_cloud->empty())
        {
            std::string map_path;

            if (req->map_path.empty())
            {
                map_path = std::string(PROJECT_SOURCE_DIR) + "/data/map.pcd";
            }
            else
            {
                map_path = req->map_path;
            }

            pcl::io::savePCDFileBinary(map_path, *map_cloud);
            res->map_path = map_path;
            res->map_size = static_cast<int64_t>(map_cloud->size());
            res->save_map_succeed = true;

            // TODO: 根据传入的路径保存
            if (req->split_map == true)
            {
                const SplitMap split_map;
                split_map.Split(*map_cloud);
            }
        }
    }
    catch (const std::exception& e)
    {
        LOG(WARNING) << "Catch save map exception: " << e.what();
        res->save_map_succeed = false;
    }

    saveKeyPathToFile();

    return true;
}

void System::saveKeyPathToFile()
{
    const std::string filename = "key_path.csv";
    std::ofstream fout{filename};
    if (!fout.is_open())
    {
        LOG(WARNING) << "Failed to open key path file: " << filename;
        return;
    }

    // std::lock_guard<std::mutex> lg(mutex_keyframes_);

    int index = 0;
    nav_msgs::msg::Path path;
    for (const auto& keyframe : keyframes_)
    {
        const auto& pose = keyframe->pose_;

        const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
        const Vec3d& t = pose.block<3, 1>(0, 3);

        ++index;

        fout << index << ", " << t.x() << ", " << t.y() << ", " << t.z() << ", " << q.x() << ", "
             << q.y() << ", " << q.z() << ", " << q.w() << "\n";
    }
}

void System::ImuMsgCallBack(sensor_msgs::msg::Imu::ConstSharedPtr imu_ptr)
{
    static Vec3d init_mean_acc = Vec3d::Zero();
    static Vec3d last_angular_velocity;
    static Eigen::Quaterniond last_orientation;
    static TimeStampUs last_timestamp;

    IMUData imu_data;
    imu_data.timestamp_ = RosTimeToUs(imu_ptr->header);
    imu_data.angular_velocity_ = RosVec3dToEigen(imu_ptr->angular_velocity);
    imu_data.linear_acceleration_ = RosVec3dToEigen(imu_ptr->linear_acceleration);

    if (ConfigParameters::Instance().fusion_method_ == kFusionLooseCoupling)
    {
        if (!has_imu_init_.load())
        {
            if (ConfigParameters::Instance().imu_has_orientation_)
            {
                imu_data.orientation_ = RosQuaternionToEigen(imu_ptr->orientation);
            }
            else
            {
                imu_data.orientation_ = Eigen::Quaterniond::Identity();
                last_angular_velocity = imu_data.angular_velocity_;
                last_orientation = Eigen::Quaterniond::Identity();
                last_timestamp = imu_data.timestamp_;
            }
            has_imu_init_.store(true);
            imu_data_searcher_ptr_->CacheData(imu_data);
            return;
        }
    }
    else
    {
        if (!has_imu_init_.load())
        {
            has_imu_init_.store(InitIMU(imu_data, init_mean_acc));

            if (has_imu_init_.load())
            {
                if (ConfigParameters::Instance().imu_has_orientation_)
                {
                    imu_data.orientation_ = RosQuaternionToEigen(imu_ptr->orientation);
                }
                else
                {
                    imu_data.orientation_ = Eigen::Quaterniond::Identity();
                    last_angular_velocity = imu_data.angular_velocity_;
                    last_orientation = Eigen::Quaterniond::Identity();
                    last_timestamp = imu_data.timestamp_;
                }

                imu_data.linear_acceleration_ = imu_data.linear_acceleration_ *
                                                ConfigParameters::Instance().gravity_norm /
                                                init_mean_acc.norm();

                imu_data_searcher_ptr_->CacheData(imu_data);
            }
            return;
        }

        imu_data.linear_acceleration_ = imu_data.linear_acceleration_ *
                                        ConfigParameters::Instance().gravity_norm /
                                        init_mean_acc.norm();
    }

    if (ConfigParameters::Instance().imu_has_orientation_)
    {
        imu_data.orientation_ = RosQuaternionToEigen(imu_ptr->orientation);
    }
    else
    {
        const Eigen::Quaterniond delta_q(
            SO3Exp((last_angular_velocity + imu_data.angular_velocity_) * 0.5 *
                   (imu_data.timestamp_ - last_timestamp) * kMicroseconds2Seconds));
        imu_data.orientation_ = last_orientation * delta_q;

        last_timestamp = imu_data.timestamp_;
        last_orientation = imu_data.orientation_;
        last_angular_velocity = imu_data.angular_velocity_;
    }

    imu_data_searcher_ptr_->CacheData(imu_data);
}

bool System::InitIMU(const IMUData& imu_data, Vec3d& init_mean_acc)
{
    static Vec3d mean_acc = Vec3d::Zero();
    static Vec3d mean_gyro = Vec3d::Zero();
    static Vec3d cov_acc = Vec3d::Zero();
    static Vec3d cov_gyro = Vec3d::Zero();
    static int N = 0;

    if (N == 0)
    {
        mean_acc = imu_data.linear_acceleration_;
        mean_gyro = imu_data.angular_velocity_;
        N = 1;
    }
    else
    {
        const auto& acc = imu_data.linear_acceleration_;
        const auto& gyro = imu_data.angular_velocity_;

        mean_acc += (acc - mean_acc) / N;
        mean_gyro += (gyro - mean_gyro) / N;

        cov_acc = cov_acc * (N - 1.0) / N +
                  (acc - mean_acc).cwiseProduct(acc - mean_acc) * (N - 1.0) / (N * N);
        cov_gyro = cov_gyro * (N - 1.0) / N +
                   (gyro - mean_gyro).cwiseProduct(gyro - mean_gyro) * (N - 1.0) / (N * N);

        N++;
    }

    init_mean_acc = mean_acc;

    if (N > 300)
    {
        N = 0;
        mean_acc = Vec3d::Zero();
        mean_gyro = Vec3d::Zero();
        cov_acc = Vec3d::Zero();
        cov_gyro = Vec3d::Zero();

        LOG(WARNING) << "IMU movement acceleration is too large, Reinitialize!";
        return false;
    }

    if (N > 200 && cov_acc.norm() < 0.05 && cov_gyro.norm() < 0.01)
    {
        ConfigParameters::Instance().gravity_vector_ =
            -mean_acc / mean_acc.norm() * ConfigParameters::Instance().gravity_norm;
        return true;
    }

    return false;
}

void System::LidarStandardMsgCallBack(sensor_msgs::msg::PointCloud2::SharedPtr cloud_ros_ptr)
{
    if (has_imu_init_.load())
    {
        std::lock_guard<std::mutex> lk(mutex_raw_cloud_deque_);

        raw_cloud_deque_.push_back(cloud_ros_ptr);
        cv_preprocessing_.notify_one();
    }
}

void System::LocalizationInitPoseMsgCallBack(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
    Eigen::Quaterniond q = RosQuaternionToEigen(msg->pose.pose.orientation);
    Vec3d t = RosPoint3dToEigen(msg->pose.pose.position);
    Mat4d pose = Mat4d::Identity();
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose.block<3, 1>(0, 3) = t;

    localization_ptr_->SetInitPose(pose);
}

// void System::LidarLivoxMsgCallBack(const
// livox_ros_driver::CustomMsg::ConstPtr& msg) {
//     using PointField = sensor_msgs::PointField;

//     static sensor_msgs::PointCloud2 cloud_ros;
//     static bool init_ros_cloud_type = false;

//     if (!init_ros_cloud_type) {
//         const auto AddField = [&](const std::string& name, uint32_t offset,
//         uint8_t datatype) {
//             sensor_msgs::PointField field;
//             field.count = 1u;
//             field.name = name;
//             field.offset = offset;
//             field.datatype = datatype;
//             cloud_ros.fields.push_back(field);
//         };

//         AddField("x", 0, PointField::FLOAT32);
//         AddField("y", cloud_ros.fields.back().offset + sizeof(float),
//         PointField::FLOAT32); AddField("z", cloud_ros.fields.back().offset +
//         sizeof(float), PointField::FLOAT32); AddField("intensity",
//         cloud_ros.fields.back().offset + sizeof(float), PointField::FLOAT32);
//         AddField("time", cloud_ros.fields.back().offset + sizeof(uint32_t),
//         PointField::UINT32); AddField("line", cloud_ros.fields.back().offset
//         + sizeof(uint8_t), PointField::UINT8); AddField("tag",
//         cloud_ros.fields.back().offset + sizeof(uint8_t), PointField::UINT8);
//         cloud_ros.is_bigendian = false;
//         cloud_ros.point_step = sizeof(float) * 4u + sizeof(uint32_t) +
//         sizeof(uint8_t) * 2u; cloud_ros.is_dense = true;

//         init_ros_cloud_type = true;
//     }

//     if (has_imu_init_.load()) {
//         size_t point_size = msg->point_num;

//         cloud_ros.header = msg->header;
//         cloud_ros.width = point_size;
//         cloud_ros.height = 1;
//         cloud_ros.row_step = point_size * cloud_ros.point_step;
//         cloud_ros.data.resize(point_size * cloud_ros.point_step);

//         pcl::PointCloud<PointXYZIRT>::Ptr cloud_in_ptr(new
//         pcl::PointCloud<PointXYZIRT>()); cloud_in_ptr->reserve(point_size);

//         unsigned char* ptr = cloud_ros.data.data();
//         for (size_t i = 0; i < point_size; ++i) {
//             *reinterpret_cast<float*>(ptr + cloud_ros.fields[0].offset) =
//             msg->points[i].x; *reinterpret_cast<float*>(ptr +
//             cloud_ros.fields[1].offset) = msg->points[i].y;
//             *reinterpret_cast<float*>(ptr + cloud_ros.fields[2].offset) =
//             msg->points[i].z; *reinterpret_cast<float*>(ptr +
//             cloud_ros.fields[3].offset) =
//                 static_cast<float>(msg->points[i].reflectivity);
//             *reinterpret_cast<uint32_t*>(ptr + cloud_ros.fields[4].offset) =
//             msg->points[i].offset_time; *reinterpret_cast<uint8_t*>(ptr +
//             cloud_ros.fields[5].offset) = msg->points[i].line;
//             *reinterpret_cast<uint8_t*>(ptr + cloud_ros.fields[6].offset) =
//             msg->points[i].tag;

//             ptr += cloud_ros.point_step;
//         }

//         sensor_msgs::PointCloud2::Ptr cloud_ros_ptr(new
//         sensor_msgs::PointCloud2); *cloud_ros_ptr = cloud_ros;

//         std::lock_guard<std::mutex> lk(mutex_raw_cloud_deque_);
//         raw_cloud_deque_.push_back(cloud_ros_ptr);
//         cv_preprocessing_.notify_one();
//     }
// }

void System::Run()
{
    if (slam_mode_ == SLAM_MODE::MAPPING)
    {
        RunMapping();
    }
    else if (slam_mode_ == SLAM_MODE::LOCALIZATION)
    {
        RunLocalization();
    }
    else
    {
        LOG(FATAL) << "SLAM Mode Error!";
    }
}

void System::RunMapping()
{
    if (ProcessMappingFrameCache())
    {
        PublishMappingKeyFramePath();
    }

    if (HasLoopClosure())
    {
        PerformLoopclosureOptimization();
        PublishMappingKeyFramePath();
    }
}

void System::RunLocalization()
{
    // publish localization path
    if (ProcessLocalizationResultCache())
    {
        PublishLocalizationPath();

        const auto current_lidar_cloud = localization_ptr_->GetCurrentLidarCloudMap();
        PublishRosCloud(localization_current_lidar_cloud_pub_, current_lidar_cloud.makeShared());
    }

    // publish local cloud map
    if (localization_ptr_->NeedUpdateLocalMapVisualization())
    {
        const auto local_cloud_map = localization_ptr_->GetLocalCloudMap();
        PublishRosCloud(localization_local_cloud_map_pub_, local_cloud_map.makeShared());
    }

    // publish global cloud map
    static bool first_global_map_visualization = true;
    if (first_global_map_visualization &&
        localization_global_cloud_map_pub_->get_subscription_count() > 0)
    {
        const auto global_cloud_map = localization_ptr_->GetGlobalCloudMap();

        if (!global_cloud_map.empty())
        {
            PublishRosCloud(localization_global_cloud_map_pub_, global_cloud_map.makeShared());
            first_global_map_visualization = false;
        }
    }
}

bool System::HasLoopClosure()
{
    std::lock_guard<std::mutex> lg(mutex_loopclosure_results_);
    return !loopclosure_result_deque_.empty();
}

bool System::ProcessLocalizationResultCache()
{
    NavStateData::Ptr nav_state_data;

    {
        std::lock_guard<std::mutex> lg(mutex_localization_results_deque_);

        if (localization_results_deque_.empty())
        {
            return false;
        }

        nav_state_data = localization_results_deque_.front();
        localization_results_deque_.pop_front();
    }

    const Mat4d pose = nav_state_data->Pose();
    PublishTF(pose, nav_state_data->timestamp_);

    const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    const Vec3d& t = pose.block<3, 1>(0, 3);
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();
    pose_stamped.pose.position.x = t.x();
    pose_stamped.pose.position.y = t.y();
    pose_stamped.pose.position.z = t.z();
    pose_stamped.header.frame_id = kRosMapFrameID;
    localization_path_.poses.emplace_back(std::move(pose_stamped));

    return true;
}

bool System::ProcessMappingFrameCache()
{
    Frame::Ptr frame;
    {
        std::lock_guard<std::mutex> lg(mutex_frame_temp_deque_);

        if (frame_temp_deque_.empty())
        {
            return false;
        }

        frame = frame_temp_deque_.front();
        frame_temp_deque_.pop_front();
    }

    // Used to determine whether keyframe needs to be added
    static Mat4d accumulated_pose = Mat4d::Identity();
    Mat4d curr_pose;
    accumulated_pose *= frame->delta_pose_;

    {
        std::lock_guard<std::mutex> lg(mutex_keyframes_);
        if (IsKeyFrame(accumulated_pose))
        {
            KeyFrame::Ptr keyframe = std::make_shared<KeyFrame>();
            keyframe->timestamp_ = frame->timestamp_;
            keyframe->cloud_cluster_ptr_ = frame->cloud_cluster_ptr_;

            if (keyframes_.empty())
            {
                keyframe->id_ = 0;
                keyframe->pose_ = accumulated_pose;
                loop_closure_optimizer_ptr_->AddVertex(keyframe->pose_, keyframe->id_, true);
            }
            else
            {
                keyframe->id_ = keyframes_.back()->id_ + 1;
                keyframe->pose_ = keyframes_.back()->pose_ * accumulated_pose;
                loop_closure_optimizer_ptr_->AddVertex(keyframe->pose_, keyframe->id_, false);
            }

            keyframes_.push_back(keyframe);

            if (keyframes_.size() >= 2u)
            {
                Mat6d info = Mat6d::Zero();
                info.diagonal() << 1.0, 1.0, 1.0, 100.0, 100.0, 100.0;
                const auto& last_keyframe = *(keyframes_.rbegin() + 1);
                const auto& curr_keyframe = *(keyframes_.rbegin());
                const Mat4d delta_pose = last_keyframe->pose_.inverse() * curr_keyframe->pose_;
                loop_closure_optimizer_ptr_->AddEdge(delta_pose, info, last_keyframe->id_,
                                                     curr_keyframe->id_);
            }

            PublishMappingKeyFrameCloud(keyframe);

            keyframe->SaveAllCloud();
            accumulated_pose.setIdentity(); // reset for next keyframe judgement
        }

        curr_pose = keyframes_.back()->pose_ * accumulated_pose;
    }

    PublishMappingFrameCloud(frame, curr_pose);
    PublishTF(curr_pose, frame->timestamp_);

    // At this point, the pointcloud in the cloud cluster is no longer needed!
    frame->cloud_cluster_ptr_->Clear();

    return true;
}

void System::PerformLoopclosureOptimization()
{
    LoopClosureResult loopclosure_result;
    {
        std::lock_guard<std::mutex> lg(mutex_loopclosure_results_);
        if (loopclosure_result_deque_.empty())
        {
            return;
        }
        else
        {
            loopclosure_result = loopclosure_result_deque_.front();
            loopclosure_result_deque_.pop_front();
        }
    }

    Mat6d info = Mat6d::Zero();
    info.diagonal() << 1.0, 1.0, 1.0, 100.0, 100.0, 100.0;
    loop_closure_optimizer_ptr_->AddEdge(loopclosure_result.match_pose_, info,
                                         loopclosure_result.candidate_keyframe_id_,
                                         loopclosure_result.loopclosure_keyframe_id_);

    loop_closure_optimizer_ptr_->Optimize(15);

    {
        std::lock_guard<std::mutex> lg(mutex_keyframes_);

        for (auto& keyframe : keyframes_)
        {
            const auto id = keyframe->id_;
            keyframe->pose_ = loop_closure_optimizer_ptr_->GetVertexEstimate(id);
        }
    }

    need_update_global_map_visualization_.store(true);
}

void System::PublishLocalizationPath()
{
    if (localization_path_pub_->get_subscription_count() > 0)
    {
        localization_path_.header.frame_id = kRosMapFrameID;
        localization_path_pub_->publish(localization_path_);
    }
}

void System::PublishMappingKeyFramePath()
{
    if (mapping_keyframe_path_cloud_pub_->get_subscription_count() > 0)
    {
        std::lock_guard<std::mutex> lg(mutex_keyframes_);

        PCLPointCloudXYZI::Ptr path_cloud(new PCLPointCloudXYZI);
        for (const auto& keyframe : keyframes_)
        {
            const auto& pose = keyframe->pose_;

            PCLPointXYZI point;
            point.x = static_cast<float>(pose(0, 3));
            point.y = static_cast<float>(pose(1, 3));
            point.z = static_cast<float>(pose(2, 3));
            point.intensity = 0;
            path_cloud->emplace_back(point);
        }

        PublishRosCloud(mapping_keyframe_path_cloud_pub_, path_cloud->makeShared());
    }

    if (mapping_keyframe_path_pub_->get_subscription_count() > 0)
    {
        std::lock_guard<std::mutex> lg(mutex_keyframes_);

        nav_msgs::msg::Path path;
        for (const auto& keyframe : keyframes_)
        {
            const auto& pose = keyframe->pose_;

            const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
            const Vec3d& t = pose.block<3, 1>(0, 3);

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();
            pose_stamped.pose.position.x = t.x();
            pose_stamped.pose.position.y = t.y();
            pose_stamped.pose.position.z = t.z();
            pose_stamped.header.frame_id = kRosMapFrameID;

            path.poses.emplace_back(std::move(pose_stamped));
        }

        path.header.frame_id = kRosMapFrameID;
        mapping_keyframe_path_pub_->publish(path);
    }
}

bool System::IsKeyFrame(const Mat4d& delta_pose) const
{
    if (keyframes_.empty())
    {
        return true;
    }

    const double delta_t = delta_pose.block<3, 1>(0, 3).norm();
    const Mat3d R_delta = delta_pose.block<3, 3>(0, 0);
    const Vec3d delta_euler = RotationMatrixToRPY(R_delta).cwiseAbs();

    if (delta_t > ConfigParameters::Instance().system_keyframe_delta_dist_ ||
        delta_euler.x() > ConfigParameters::Instance().system_keyframe_delta_rotation_ ||
        delta_euler.y() > ConfigParameters::Instance().system_keyframe_delta_rotation_ ||
        delta_euler.z() > ConfigParameters::Instance().system_keyframe_delta_rotation_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void System::PublishMappingKeyFrameCloud(const KeyFrame::Ptr& keyframe)
{
    CHECK_NOTNULL(keyframe);
    CHECK_NOTNULL(keyframe->cloud_cluster_ptr_);
    if (keyframe->cloud_cluster_ptr_->ordered_cloud_.empty())
    {
        return;
    }

    sensor_msgs::msg::PointCloud2 cloud_ros;
    PCLPointCloudXYZI::Ptr cloud_trans(new PCLPointCloudXYZI);

    *cloud_trans = keyframe->cloud_cluster_ptr_->ordered_cloud_;
    cloud_trans = TransformPointCloud(cloud_trans, keyframe->pose_);
    PublishRosCloud(mapping_curr_keyframe_cloud_pub_, cloud_trans);
}

void System::PublishMappingFrameCloud(const Frame::Ptr& frame, const Mat4d& pose)
{
    CHECK_NOTNULL(frame);

    PCLPointCloudXYZI::Ptr cloud_trans(new PCLPointCloudXYZI);

    if (!frame->cloud_cluster_ptr_->ordered_cloud_.empty())
    {
        cloud_trans = VoxelGridCloud(frame->cloud_cluster_ptr_->ordered_cloud_, 0.5);
        cloud_trans = TransformPointCloud(cloud_trans, pose);
        PublishRosCloud(mapping_curr_cloud_pub_, cloud_trans);
    }

    if (!frame->cloud_cluster_ptr_->corner_cloud_.empty())
    {
        *cloud_trans = frame->cloud_cluster_ptr_->corner_cloud_;
        cloud_trans = TransformPointCloud(cloud_trans, pose);
        PublishRosCloud(mapping_curr_corner_cloud_pub_, cloud_trans);
    }

    if (!frame->cloud_cluster_ptr_->planar_cloud_.empty())
    {
        *cloud_trans = frame->cloud_cluster_ptr_->planar_cloud_;
        cloud_trans = TransformPointCloud(cloud_trans, pose);
        PublishRosCloud(mapping_curr_planer_cloud_pub_, cloud_trans);
    }
}

void System::PublishTF(const Mat4d& pose, TimeStampUs timestamp)
{
    const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    const Vec3d& p = pose.block<3, 1>(0, 3);

    // tf_broadcaster_->sendTransform()

    tf2::Transform transform(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()),
                             tf2::Vector3(p.x(), p.y(), p.z()));

    geometry_msgs::msg::TransformStamped tfStamped;
    tfStamped.transform = tf2::toMsg(transform);
    tfStamped.header.frame_id = kRosMapFrameID;
    tfStamped.header.stamp = rclcpp::Time(static_cast<double>(timestamp) * kMicroseconds2Seconds);
    tfStamped.child_frame_id = kRosLidarFrameID;

    tf_broadcaster_->sendTransform(tfStamped);

    // tf_broadcaster_->sendTransform(
    //     tf::StampedTransform(tf2::Transform(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()),
    //                                         tf::Vector3(p.x(), p.y(), p.z())),
    //                          rclcpp::Time(static_cast<double>(timestamp) *
    //                          kMicroseconds2Seconds), kRosMapFrameID, kRosLidarFrameID));
}

void System::VisualizeGlobalMap()
{
    CHECK(slam_mode_ == SLAM_MODE::MAPPING)
        << "VisualizeGlobalMap can only be used in mapping mode!";

    LOG(INFO) << "\033[1;32m----> Global Map Visualization Started.\033[0m";
    PCLPointCloudXYZI::Ptr global_map(new PCLPointCloudXYZI);
    KeyFrame::ID last_frame_id = -1;

    float voxel_filter_size =
        ConfigParameters::Instance().system_global_map_visualization_resolution_;

    rclcpp::Rate rate(0.5);
    while (rclcpp::ok())
    {
        rate.sleep();
        if (mapping_global_map_cloud_pub_->get_subscription_count() == 0)
        {
            continue;
        }

        if (need_update_global_map_visualization_.load())
        {
            need_update_global_map_visualization_.store(false);
            global_map->clear();
            last_frame_id = -1;
        }

        std::vector<KeyFrame::Ptr> keyframes;
        {
            std::lock_guard<std::mutex> lg(mutex_keyframes_);

            if (keyframes_.empty() || (last_frame_id + 1 >= keyframes_.back()->id_))
            {
                continue;
            }

            for (int i = last_frame_id + 1; i <= keyframes_.back()->id_; ++i)
            {
                keyframes.push_back(keyframes_[i]);
            }
            last_frame_id = keyframes_.back()->id_;
        }

        if (!keyframes.empty())
        {
            PCLPointCloudXYZI temp_local_cloud;
            for (const auto& keyframe : keyframes)
            {
                auto cloud = keyframe->LoadOrderedCloud();
                temp_local_cloud += *TransformPointCloud(VoxelGridCloud(*cloud, voxel_filter_size),
                                                         keyframe->pose_);
            }

            *global_map += temp_local_cloud;
            global_map = VoxelGridCloud(*global_map, voxel_filter_size);

            PublishRosCloud(mapping_global_map_cloud_pub_, global_map);
        }
    }
}
