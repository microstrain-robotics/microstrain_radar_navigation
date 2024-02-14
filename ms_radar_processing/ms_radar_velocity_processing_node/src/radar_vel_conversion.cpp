#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
using std::placeholders::_1;

namespace ms_radar_velocity_processing 
{
    class RadarVelTransform : public rclcpp::Node
    {
        public:
            RadarVelTransform() : Node("radar_vel_transform")
            {
                subscription_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
                    "/cv7_ins/ext/velocity_body",10,std::bind(&RadarVelTransform::transform_callback,this,_1));

                publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/radar_vel_transform/base",10);
            }

        private:
            void transform_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr data) const
            {
                tf2_ros::Buffer tfBuffer;
                tf2_ros::TransformListener tfListener(tfBuffer);

                transformStamped = tfBuffer.lookupTransform("/gps","/radar_link",ros::Time(0));
                publisher_->publish(transformStamped);
                
            }

            rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr subscription_;
            rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_;

    };

}

int main (int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RadarVelTransform>());
    rclcpp::shutdown();
    return 0;
}