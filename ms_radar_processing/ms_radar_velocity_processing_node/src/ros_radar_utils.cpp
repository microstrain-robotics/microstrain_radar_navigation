//
// Created by davidrobbins on 11/28/23.
//
#include <optional>

#include "ms_radar_velocity_processing_node/ros_radar_utils.h"

#include "rclcpp/rclcpp.hpp"

namespace ms_radar_velocity_processing_node
{
    ms_radar_velocity_processing::RadarPointCloud convert_from_ros_message(const sensor_msgs::msg::PointCloud& point_cloud_message)
    {
        ms_radar_velocity_processing::RadarPointCloud output_radar_point_cloud;

        output_radar_point_cloud.timestamp = double(point_cloud_message.header.stamp.sec) + double(point_cloud_message.header.stamp.nanosec)*1e-9;

        for (const auto& channel : point_cloud_message.channels)
        {
            if (channel.name == "Doppler") 
                output_radar_point_cloud.speed.insert(output_radar_point_cloud.speed.end(), channel.values.begin(), channel.values.end());
            
            if (channel.name == "Power")
                output_radar_point_cloud.power.insert(output_radar_point_cloud.power.end(), channel.values.begin(), channel.values.end());
        }

        for (const auto& ros_point : point_cloud_message.points)
        {
            output_radar_point_cloud.points.emplace_back(ros_point.x, ros_point.y, ros_point.z);
        }
        if (output_radar_point_cloud.points.size() != output_radar_point_cloud.speed.size())
            throw std::runtime_error("Failed to extract radar speed from input message");

        if (output_radar_point_cloud.points.size() != output_radar_point_cloud.power.size())
            throw std::runtime_error("Failed to extract radar power from input message");

        return output_radar_point_cloud;
    }
}
