//
// Created by Zhang Zhimeng on 23-11-26.
//

#include "slam/config_parameters.h"

#include <mutex>

std::unique_ptr<ConfigParameters> ConfigParameters::instance_ = nullptr;

ConfigParameters& ConfigParameters::Instance()
{
    if (instance_ == nullptr)
    {
        static std::once_flag flag;
        std::call_once(flag, [&] { instance_.reset(new (std::nothrow) ConfigParameters()); });
    }

    return *instance_;
}

std::string ConfigParameters::toString() const
{
    nlohmann::ordered_json mainJ, topicJ;
    topicJ["lidar_topic"] = lidar_topic_;
    topicJ["imu_topic"] = imu_topic_;

    mainJ["topic_names"] = topicJ;

    return mainJ.dump();
}
