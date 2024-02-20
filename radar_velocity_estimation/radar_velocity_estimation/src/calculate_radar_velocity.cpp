//
// Created by davidrobbins on 11/28/23.
//

#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"

#include "radar_velocity_estimation/calculate_radar_velocity.h"

#include "RadialVelocityFactor.h"

namespace radar_velocity_estimation
{
    std::tuple<bool, Eigen::Vector3d, Eigen::Matrix3d> caluclate_radar_velocity_with_covariance(const RadarPointCloud& radar_point_cloud, const RadarVelocitySettings& radar_velocity_settings)
    {
        Eigen::Vector3d velocity_estimate = Eigen::Vector3d::Zero();
        Eigen::Matrix3d velocity_covariance = Eigen::Matrix3d::Zero();
        bool solution_valid = false;

        if (radar_point_cloud.points.size() < 3)
        {
            return {solution_valid, velocity_estimate, velocity_covariance};
        }

        gtsam::Key velocity_key = 0;

        gtsam::NonlinearFactorGraph factors;

        gtsam::Values states;
        states.insert(velocity_key, gtsam::Vector3());

        for (size_t i = 0; i<radar_point_cloud.points.size(); i++)
        {
            // Check minimum distance threshold
            if (radar_point_cloud.points[i].norm() < radar_velocity_settings.min_distance)
                {
                    continue;
                }

            auto radial_velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(1, .25);
            auto robust_radial_velocity_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(3), radial_velocity_noise_model);
            auto radar_factor = boost::make_shared<BodyframeVelocityFactor>(velocity_key, radar_point_cloud.points[i], -1 * radar_point_cloud.speed[i], robust_radial_velocity_noise_model);
            factors.add(radar_factor);
        }

        if( factors.size()==0)
        {
            return {solution_valid, velocity_estimate, velocity_covariance};
        }


        try
        {
            gtsam::LevenbergMarquardtOptimizer optimizer(factors, states);
            gtsam::Values solution = optimizer.optimize();

            velocity_estimate = solution.at<gtsam::Vector3>(velocity_key);

            gtsam::Marginals marginals(factors, solution);
            velocity_covariance = marginals.marginalCovariance(velocity_key);
            solution_valid = true;
        }
        catch (const gtsam::IndeterminantLinearSystemException& e)
        {}

        double target_x_covariance = pow(.15, 2);
        double covariance_scaling = target_x_covariance/velocity_covariance(0,0);
        velocity_covariance *= covariance_scaling;

        return {solution_valid, velocity_estimate, velocity_covariance};
    }
}
