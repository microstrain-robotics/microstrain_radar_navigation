//
// Created by davidrobbins on 11/28/23.
//

#ifndef ROS_RADAR_UTILS_H
#define ROS_RADAR_UTILS_H

#include "sensor_msgs/msg/point_cloud.hpp"

#include "ms_radar_velocity_processing/data.h"

namespace ms_radar_velocity_processing_node
{
    ms_radar_velocity_processing::RadarPointCloud convert_from_ros_message(const sensor_msgs::msg::PointCloud& point_cloud_message);

    template<int rows, int cols>
    std::array<double, rows * cols> from_eigen_covariance(const Eigen::Matrix<double, rows, cols> covariance)
    {
        std::array<double, rows * cols> covariance_vector;
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                covariance_vector[cols * i + j] = covariance(i, j);
        return covariance_vector;
    }
}

#endif //ROS_RADAR_UTILS_H
