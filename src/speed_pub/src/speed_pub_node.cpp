#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

using namespace std::chrono_literals;

class SpeedPubNode : public rclcpp::Node
{
    public:
        SpeedPubNode()
        : Node("speed_pub_node")
        {
            publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("target_speed", 10);
            timer_ = this->create_wall_timer(
            20ms, std::bind(&SpeedPubNode::timer_callback, this));
        }

    private:
    void timer_callback()
    {
        auto message = std_msgs::msg::Int32MultiArray();
        message.data = {200, 200};
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d', '%d'", message.data[0], message.data[1]);
        publisher_->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedPubNode>());
    rclcpp::shutdown();
    return 0;
}