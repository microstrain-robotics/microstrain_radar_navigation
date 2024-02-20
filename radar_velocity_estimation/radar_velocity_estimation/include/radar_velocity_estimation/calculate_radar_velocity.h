//
// Created by davidrobbins on 11/28/23.
//

#ifndef CALCULATE_RADAR_VELOCITY_H
#define CALCULATE_RADAR_VELOCITY_H

#include <tuple>

#include "data.h"

namespace radar_velocity_estimation
{
    struct RadarVelocitySettings
    {
        double min_distance = 3.0;
    };

    std::tuple<bool, Eigen::Vector3d, Eigen::Matrix3d> caluclate_radar_velocity_with_covariance(const RadarPointCloud& radar_point_cloud, const RadarVelocitySettings& radar_velocity_settings);
}


#endif //CALCULATE_RADAR_VELOCITY_H
