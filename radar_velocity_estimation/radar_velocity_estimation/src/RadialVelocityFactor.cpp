//
// Created by davidrobbins on 11/28/23.
//

#include "RadialVelocityFactor.h"

namespace radar_velocity_estimation {

    gtsam::Vector BodyframeVelocityFactor::evaluateError(const gtsam::Vector3& velocity_estimate,boost::optional<gtsam::Matrix&> H1) const
    {
        gtsam::Vector1 error(_direction_unit_vector.transpose()*velocity_estimate - _speed_measurement);
        if (H1)
        {
            (*H1) = _direction_unit_vector.transpose();
        }

        return error;
    }
} // radar_velocity_estimation
