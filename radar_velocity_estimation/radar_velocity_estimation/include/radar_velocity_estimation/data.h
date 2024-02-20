//
// Created by davidrobbins on 11/28/23.
//

#ifndef RADAR_VELOCITY_ESTIMATION_DATA_H
#define RADAR_VELOCITY_ESTIMATION_DATA_H

#include <Eigen/Core>

namespace radar_velocity_estimation
{

    struct RadarPointCloud
    {
        double timestamp;
        std::vector<Eigen::Vector3d> points;
        std::vector<double> speed;
        std::vector<double> power;
    };

}

#endif //RADAR_VELOCITY_ESTIMATION_DATA_H
