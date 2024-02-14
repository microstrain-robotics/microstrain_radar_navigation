//
// Created by davidrobbins on 11/28/23.
//

#ifndef MS_RADAR_VELOCITY_PROCESSING_DATA_H
#define MS_RADAR_VELOCITY_PROCESSING_DATA_H

#include <Eigen/Core>

namespace ms_radar_velocity_processing
{

    struct RadarPointCloud
    {
        double timestamp;
        std::vector<Eigen::Vector3d> points;
        std::vector<double> speed;
        std::vector<double> power;
    };

}

#endif //MS_RADAR_VELOCITY_PROCESSING_DATA_H
