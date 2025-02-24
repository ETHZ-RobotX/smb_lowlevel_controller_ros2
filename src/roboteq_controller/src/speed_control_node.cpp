#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "roboteq_controller/inter_process_communication.h"

using namespace std::chrono_literals;

class SpeedControlNode : public rclcpp::Node
{
    public:
        SpeedControlNode() : Node("speed_control_node"), port_("/dev/ttyUSB0"), baudrate_(115200), motor_1_channel_(1), motor_2_channel_(2)
        {
            this->declare_parameter<std::string>("port", port_);
            this->declare_parameter<int>("baudrate", baudrate_);
            this->declare_parameter<int>("motor_1_channel", motor_1_channel_);
            this->declare_parameter<int>("motor_2_channel", motor_2_channel_);

            connect();
            RCLCPP_INFO(this->get_logger(), "Attempting to create a publisher");
            // Create a publisher - publishes the motor speed in RPM
            wheel_speed_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("wheel_speed", 10);
            RCLCPP_INFO(this->get_logger(), "Publisher created");
            wheel_speed_subscriber_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("target_speed", 10, std::bind(&SpeedControlNode::set_speed, this, std::placeholders::_1));
            publish_timer_ = this->create_wall_timer(10ms, std::bind(&SpeedControlNode::get_speed, this));
            RCLCPP_INFO(this->get_logger(), "Timer created");
            freq_timer_ = this->create_wall_timer(1s, std::bind(&SpeedControlNode::stream_rate, this));
            
        }

        ~SpeedControlNode()
        {
            disconnect();
        }
    
    private:
        int state_ = 0;
        int motor_1_channel_;
        int motor_2_channel_;
        int baudrate_;
        std::string port_;
        std::unique_ptr<serial::Serial> serial_;
        std::string prefix_ = "D=";
        std::string delimiter_ = ":";
        int prefix_length_ = prefix_.length();
        int count_ = 0;

        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr wheel_speed_publisher_;
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr wheel_speed_subscriber_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::TimerBase::SharedPtr freq_timer_;

        void connect()
        {
            try
            {
                serial_ = std::make_unique<serial::Serial>(port_, baudrate_, serial::Timeout::simpleTimeout(1000));
                RCLCPP_INFO(this->get_logger(), "Trying to open serial port %s", port_.c_str());
                // serial_->open();
                if(check_connection())
                {
                    RCLCPP_INFO(this->get_logger(), "Serial port initialized");
                    state_ = 1;
                    

                    std::string echo_query = "^ECHOF 1\r\n";
                    RCLCPP_INFO(this->get_logger(), "Sending Query %s", echo_query.c_str());
                    serial_->write(echo_query);
                    std::string speed_query = "/\"d=\",\":\"?S " + std::to_string(motor_1_channel_) + "_?S " + std::to_string(motor_2_channel_) + "_# 5\r\n";
                    RCLCPP_INFO(this->get_logger(), "Sending Query %s", speed_query.c_str());
                    serial_->write(speed_query);
                    
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Serial port not open");
                    // state_ = 0;
                }
            }
            catch(serial::IOException& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Unable to open port");
                // state_ = 0;
            }
        }

        void disconnect()
        {
            if(check_connection())
            {
                serial_->close();
                RCLCPP_INFO(this->get_logger(), "Serial port closed");
                state_ = 0;
            }
        }

        bool check_connection()
        {
            return serial_ && serial_->isOpen();
        }

        void set_speed(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
        {
            if (check_connection())
            {
                int target_speed_1 = msg->data[0];
                int target_speed_2 = msg->data[1];
                std::string speed_command = "!S 1 " + std::to_string(target_speed_1) + "\r" +
                                            "!S 2 " + std::to_string(target_speed_2) + "\r\n";

                serial_->write(speed_command);

                // Wait for '+' acknowledgment
                std::string response;
                auto start_time = std::chrono::steady_clock::now();
                bool ack_received = false;

                while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(500)) // 500 ms timeout
                {
                    if (serial_->available())
                    {
                        char c;
                        serial_->read(reinterpret_cast<uint8_t*>(&c), 1);  // Cast char* to uint8_t*
                        response += c;

                        if (c == '+') // Check for acknowledgment
                        {
                            ack_received = true;
                            break;
                        }
                    }
                }

                if (ack_received)
                {
                    // RCLCPP_INFO(this->get_logger(), "Acknowledgment received: +");
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "No acknowledgment received within timeout");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Serial port not open");
            }
        }


        void get_speed()
        {
            Speed speed;
            // RCLCPP_INFO(this->get_logger(), "Getting speed");
            if(check_connection())
            {
                serial_->flushInput();
                std::string response = serial_->readline(100, "\r");
                // RCLCPP_INFO(this->get_logger(), "Received: %s", response.c_str());

                try
                {
                    speed = parse_string(response);
                    // RCLCPP_INFO(this->get_logger(), "Left: %d, Right: %d", speed.motor_1_speed, speed.motor_2_speed);
                    std_msgs::msg::Int32MultiArray msg;
                    msg.data = {speed.motor_1_speed, speed.motor_2_speed};
                    wheel_speed_publisher_->publish(msg);
                    count_++;
                    // RCLCPP_INFO(this->get_logger(), "Speed published");
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error parsing speed: %s", e.what());
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Serial port not open");
            }
        }

        struct Speed
        {
            int motor_1_speed;
            int motor_2_speed;
        };

        Speed parse_string(std::string str)
        {
            Speed speed;
            std::string token;
            token = str.substr(prefix_length_, str.find(delimiter_) - prefix_length_);
            speed.motor_1_speed = std::stoi(token);
            str.erase(0, token.length() + prefix_length_ + delimiter_.length());
            speed.motor_2_speed = std::stoi(str);
            return speed;
        }

        void stream_rate()
        {
            if(check_connection())
            {
                // RCLCPP_INFO(this->get_logger(), "Stream rate: %d", count_);
                count_ = 0;
            }
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedControlNode>());
    rclcpp::shutdown();
    return 0;
} 