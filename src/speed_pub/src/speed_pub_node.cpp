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
            rc_input_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("rc_input", 10, std::bind(&SpeedPubNode::set_target_speed, this, std::placeholders::_1));
            timer_ = this->create_wall_timer(
            10ms, std::bind(&SpeedPubNode::timer_callback, this));
        }

    private:
    double left_speed = 0.0;
    double right_speed = 0.0;
    double wheel_base = 1.0;
    void set_target_speed(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        left_speed = msg->linear.x + wheel_base*msg->angular.z;
        right_speed = msg->linear.x - wheel_base*msg->angular.z;
        right_speed = -right_speed;
        RCLCPP_INFO(this->get_logger(), "Received: '%f', '%f'", left_speed, right_speed);
    }

    void timer_callback()
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {left_speed, right_speed};
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f'", msg.data[0], msg.data[1]);
        target_speed_publisher_->publish(msg);
    }
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_speed_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr rc_input_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedPubNode>());
    rclcpp::shutdown();
    return 0;
}