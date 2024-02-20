//
// Created by davidrobbins on 11/28/23.
//

#ifndef RADIALVELOCITYFACTOR_H
#define RADIALVELOCITYFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>

namespace radar_velocity_estimation {

    class BodyframeVelocityFactor  : public gtsam::NoiseModelFactor1<gtsam::Vector3>
    {
    public:

        typedef boost::shared_ptr<BodyframeVelocityFactor> shared_ptr;

        BodyframeVelocityFactor(gtsam::Key velocity_key, const Eigen::Vector3d& point, double speed,
                                                 const gtsam::SharedNoiseModel& model) : gtsam::NoiseModelFactor1<gtsam::Vector3>(model, velocity_key),
                                                                                         _direction_unit_vector(point.normalized()), _speed_measurement(speed)
        {
        }

        gtsam::Vector evaluateError(const gtsam::Vector3& velocity_estimate,
                                    boost::optional<gtsam::Matrix&> H1 = boost::none) const override;

    protected:

        Eigen::Vector3d _direction_unit_vector;
        double _speed_measurement;
    };

} // radar_velocity_estimation

#endif //RADIALVELOCITYFACTOR_H
