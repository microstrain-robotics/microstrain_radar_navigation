#include "rclcpp/rclcpp.hpp"

#include "radar_velocity_estimation_node/RadarVelocityNode.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto radar_velocity_node = std::make_shared<radar_velocity_estimation_node::RadarVelocityNode>();
    executor.add_node(radar_velocity_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
