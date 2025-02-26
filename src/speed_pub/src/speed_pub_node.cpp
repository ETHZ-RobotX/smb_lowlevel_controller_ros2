#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class SpeedPubNode : public rclcpp::Node
{
    public:
        SpeedPubNode()
        : Node("speed_pub_node")
        {
            target_speed_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("target_speed", 10);
            rc_input_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("rc_input", 10, std::bind(&SpeedPubNode::wheel_speed_calc, this, std::placeholders::_1));
            
            timer_ = this->create_wall_timer(
            50ms, std::bind(&SpeedPubNode::timer_callback, this));
        }

    private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_speed_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rc_input_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    double left_speed_ = 0;
    double right_speed_ = 0;
    double wheel_base = 2.0;

    void wheel_speed_calc(const geometry_msgs::msg::Twist::SharedPtr rc_input_msg)
    {
        left_speed_ = rc_input_msg->linear.x + rc_input_msg->angular.z * wheel_base / 2;
        right_speed_ = rc_input_msg->linear.x - rc_input_msg->angular.z * wheel_base / 2;
    }

    void timer_callback()
    {
        std_msgs::msg::Float64MultiArray target_speed_msg;
        geometry_msgs::msg::Twist rc_input_msg;

        target_speed_msg.data = {left_speed_, right_speed_};
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f'", target_speed_msg.data[0], target_speed_msg.data[1]);
        target_speed_publisher_->publish(target_speed_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedPubNode>());
    rclcpp::shutdown();
    return 0;
}