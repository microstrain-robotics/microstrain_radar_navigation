//
// Created by davidrobbins on 11/28/23.
//

#include "ms_radar_velocity_processing_node/RadarVelocityNode.h"
#include "ms_radar_velocity_processing_node/topics.h"
#include "ms_radar_velocity_processing_node/ros_radar_utils.h"

#include "ms_radar_velocity_processing/calculate_radar_velocity.h"


namespace ms_radar_velocity_processing_node {
    RadarVelocityNode::RadarVelocityNode(const std::string& node_name) : rclcpp::Node(node_name)
    {
        // Parameters
        this->declare_parameter("min_point_distance", 3.0);

        // Subscribers
        _point_cloud_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud>(topics::ROS_INPUT_RADAR_TOPIC_NAME, rclcpp::SensorDataQoS(), std::bind(&RadarVelocityNode::handle_input_point_cloud, this, std::placeholders::_1));

        // Publishers
        _radar_velocity_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(topics::ROS_OUTPUT_RADAR_VELOCITY_TOPIC_NAME, 10);
        _radar_velocity_viz_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>(topics::ROS_OUTPUT_RADAR_VELOCITY_VIZ_TOPIC_NAME, 10);
    }

    void RadarVelocityNode::handle_input_point_cloud(sensor_msgs::msg::PointCloud::SharedPtr point_cloud_msg)
    {
        ms_radar_velocity_processing::RadarPointCloud radar_point_cloud = convert_from_ros_message(*point_cloud_msg);

        // Get configuration
        ms_radar_velocity_processing::RadarVelocitySettings radar_velocity_settings;
        radar_velocity_settings.min_distance = get_min_point_distance();

        auto [solution_valid, velocity, velocity_covariance] = ms_radar_velocity_processing::caluclate_radar_velocity_with_covariance(radar_point_cloud, radar_velocity_settings);

        if (!solution_valid)
        {
            RCLCPP_WARN(get_logger(), "Failed to calculate radar velocity estimate");
            return;
        }
        
        geometry_msgs::msg::TwistWithCovarianceStamped output_twist_with_cov_msg;
        output_twist_with_cov_msg.header = point_cloud_msg->header;

        output_twist_with_cov_msg.twist.twist.linear.x = velocity[0];
        output_twist_with_cov_msg.twist.twist.linear.y = velocity[1];
        output_twist_with_cov_msg.twist.twist.linear.z = velocity[2];

        Eigen::Matrix<double,6,6> covariance = Eigen::Matrix<double,6,6>::Identity() * -1;
        covariance.block<3,3>(0,0) = velocity_covariance;

        output_twist_with_cov_msg.twist.covariance = from_eigen_covariance(covariance);

        _radar_velocity_publisher->publish(output_twist_with_cov_msg);

        geometry_msgs::msg::TwistStamped output_twist_msg;
        output_twist_msg.header = point_cloud_msg->header;
        output_twist_msg.twist = output_twist_with_cov_msg.twist.twist;

        _radar_velocity_viz_publisher->publish(output_twist_msg);
    }

    void RadarVelocityNode::reset(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
    }
} // ms_radar_velocity_processing_node
