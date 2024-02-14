#include "rclcpp/rclcpp.hpp"

#include "ms_radar_velocity_processing_node/RadarVelocityNode.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto radar_velocity_node = std::make_shared<ms_radar_velocity_processing_node::RadarVelocityNode>();
    executor.add_node(radar_velocity_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
