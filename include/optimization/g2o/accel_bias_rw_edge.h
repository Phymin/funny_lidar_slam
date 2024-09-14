//
// Created by Zhang Zhimeng on 24-1-17.
//

#ifndef FUNNY_LIDAR_SLAM_ACCEL_BIAS_RW_EDGE_H
#define FUNNY_LIDAR_SLAM_ACCEL_BIAS_RW_EDGE_H

#include "common/data_type.h"
#include "optimization/g2o/vertex_type.h"

#include <g2o/core/base_binary_edge.h>

class EdgeAccelBiasRW final : public g2o::BaseBinaryEdge<3, Vec3d, VertexBiasAccel, VertexBiasAccel> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeAccelBiasRW() = default;

    bool read(std::istream &is) override {
        return false;
    }

    bool write(std::ostream &os) const override {
        return false;
    }

    void computeError() override;

    void linearizeOplus() override;

    Mat6d GetPosteriorInformation();
};

#endif //FUNNY_LIDAR_SLAM_ACCEL_BIAS_RW_EDGE_H
