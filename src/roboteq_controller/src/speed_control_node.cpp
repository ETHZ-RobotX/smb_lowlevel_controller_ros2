#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>

#include "roboteq_controller/inter_process_communication.h"

using namespace std::chrono_literals;

class SpeedControlNode : public rclcpp::Node
{
    public:
        SpeedControlNode() : Node("speed_control_node"), port_("/dev/ttyUSB0"), baudrate_(115200), motor_1_channel_(1), motor_2_channel_(2), last_command_time_(this->now())
        {
            this->declare_parameter<std::string>("port", port_);
            this->declare_parameter<int>("baudrate", baudrate_);
            this->declare_parameter<int>("motor_1_channel", motor_1_channel_);
            this->declare_parameter<int>("motor_2_channel", motor_2_channel_);
            this->declare_parameter<int>("send_cmd", 1);

            connect();
            send_cmd_flag_ = this->get_parameter("send_cmd").as_int();
            RCLCPP_INFO(this->get_logger(), "Serial control flag: %d", send_cmd_flag_);
            // Create a publisher - publishes the motor speeds in RPM
            wheel_speed_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_speed", 10);
            // Create a publisher - publishes the RC input as a Twist message
            rc_input_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            // Create a publisher - publishes a dummy message - to check the frequency of the node
            dummy_publisher_ = this->create_publisher<std_msgs::msg::Int32>("dummy", 10);
            // Creating a subscriber - to receive the target speed
            wheel_speed_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("wheel_joint_commands", 10, std::bind(&SpeedControlNode::set_speed, this, std::placeholders::_1));
            // Creating timer - for publoshing topics
            publish_timer_ = this->create_wall_timer(10ms, std::bind(&SpeedControlNode::publish_info, this));
            // Creating timer - to read motor info (speed + rc inputs) from the motor controller
            read_timer_ = this->create_wall_timer(5ms, std::bind(&SpeedControlNode::read_info, this));
            // Creating timer - to check the stream rate
            flush_timer_ = this->create_wall_timer(100ms, std::bind(&SpeedControlNode::flush, this));
            // Creating timer - to check if the node is receiving speed commands
            // watchdog_timer_ = this->create_wall_timer(500ms, std::bind(&SpeedControlNode::watchdog_check, this));
            
        }

        ~SpeedControlNode()
        {
            stop_motors();
            disconnect();
        }
    
    private:
        int state_ = 0; // 0 - Disconnected, 1 - Connected
        bool emergency_stop_ = false; // sores emergency stop switch value (from the RC)
        int motor_1_channel_;
        int motor_2_channel_;
        int send_cmd_flag_;
        int baudrate_;
        std::string port_;
        std::unique_ptr<serial::Serial> serial_;
        std::string prefix_ = "D=";  //Used for parsing the response
        std::string delimiter_ = ":";
        int prefix_length_ = prefix_.length();
        int count_ = 0;
        std::string serial_response_;
        rclcpp::TimerBase::SharedPtr read_timer_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_speed_publisher_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_speed_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rc_input_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dummy_publisher_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::TimerBase::SharedPtr flush_timer_;

        rclcpp::Time last_command_time_;

        void connect()
        {
            try
            {
                serial_ = std::make_unique<serial::Serial>(port_, baudrate_, serial::Timeout::simpleTimeout(1000));
                RCLCPP_INFO(this->get_logger(), "Trying to open serial port %s", port_.c_str());
                if(check_connection())
                {
                    RCLCPP_INFO(this->get_logger(), "Serial port initialized");
                    state_ = 1;

                    // Stop all ongoing communication to and from the controller
                    std::string stop_query = "#\r\n";
                    RCLCPP_INFO(this->get_logger(), "Sending Query %s", stop_query.c_str());
                    serial_->write(stop_query);                    

                    // Turn off echoing
                    std::string echo_query = "^ECHOF 1\r\n";
                    RCLCPP_INFO(this->get_logger(), "Sending Query %s", echo_query.c_str());
                    serial_->write(echo_query);

                    // Initialize data stream - to get motor speeds and RC inputs
                    std::string speed_query = "/\"d=\",\":\"?S " + std::to_string(motor_1_channel_) + "_?S " + std::to_string(motor_2_channel_) + "_?PI 1_?PI 2_# 5\r\n";
                    RCLCPP_INFO(this->get_logger(), "Sending Query %s", speed_query.c_str());
                    serial_->write(speed_query);
                    
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Serial port not open");
                }
            }
            catch(serial::IOException& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Unable to open port");
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

        void set_speed(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            if(send_cmd_flag_ != 0)
            {
                
                if (check_connection())
                {
                    int target_speed_1 = llround(msg->data[0]);
                    int target_speed_2 = llround(msg->data[1]);
                    last_command_time_ = this->now();
                    // Santity check
                    if (target_speed_1 > 1000)
                    {
                        target_speed_1 = 1000;
                    }
                    else if (target_speed_1 < -1000)
                    {
                        target_speed_1 = -1000;
                    }
                    if (target_speed_2 > 1000)
                    {
                        target_speed_2 = 1000;
                    }
                    else if (target_speed_2 < -1000)
                    {
                        target_speed_2 = -1000;
                    }
                    std::string speed_command = "!G 1 " + std::to_string(target_speed_1) + "\r" + "!G 2 " + std::to_string(target_speed_2) + "\r\n";

                    serial_->write(speed_command);
                    // Flush input buffer to prevent lag
                    // serial_->flush();
                    // serial_->flushInput();
                    
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Serial port not open");
                }
            }
        }

        void read_info()
        {
            if(check_connection())
            {
                
                std::string response = serial_->readline(20, "\r");
                // Check if it starts with prefix
                if(response.find(prefix_) == 0)
                {
                    serial_response_ = response;
                    count_++;
                    std_msgs::msg::Int32 dummy_msg;
                    dummy_msg.data = count_;
                    dummy_publisher_->publish(dummy_msg);
                }
                else
                {
                    // RCLCPP_ERROR(this->get_logger(), "Invalid response: %s", response.c_str());
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Serial port not open");
            }
        }
        
        void publish_info()
        {
            motor_info_struct motor_info;
            if(check_connection())
            {

                try
                {
                    // Publish to all topics
                    motor_info = parse_string(serial_response_);
                    std_msgs::msg::Float64MultiArray wheel_speed_msg;
                    geometry_msgs::msg::Twist rc_input_msg;
                    wheel_speed_msg.data = {motor_info.motor_1_speed, motor_info.motor_2_speed};
                    rc_input_msg.linear.x = 33*(motor_info.pulse_1 - motor_info.pulse_2)/3000.0; //Max linear velocity is 5 m/s
                    rc_input_msg.angular.z = (3000 - motor_info.pulse_1 - motor_info.pulse_2)/3000.0; //Max angular velocity is 1 rad/s
                    rc_input_publisher_->publish(rc_input_msg);
                    wheel_speed_publisher_->publish(wheel_speed_msg);
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

        struct motor_info_struct
        {
            int motor_1_speed;
            int motor_2_speed;
            int pulse_1;
            int pulse_2;
        };

        motor_info_struct parse_string(std::string str)
        {
            // Parses incoming data stream into the motor speeds and rc inputs
            motor_info_struct motor_info;
            std::string token;
            token = str.substr(prefix_length_, str.find(delimiter_) - prefix_length_);
            motor_info.motor_1_speed = std::stod(token);
            str.erase(0, token.length() + prefix_length_ + delimiter_.length());

            token = str.substr(0, str.find(delimiter_));
            motor_info.motor_2_speed = std::stod(token);
            str.erase(0, token.length() + delimiter_.length());

            token = str.substr(0, str.find(delimiter_));
            motor_info.pulse_1 = std::stod(token);
            str.erase(0, token.length() + delimiter_.length());

            motor_info.pulse_2 = std::stod(str);
            return motor_info;
        }

        void flush()
        {
            if(check_connection())
            {
                serial_->flush();
                serial_->flushInput();
            }
        }

        void stop_motors()
        {
            if (serial_ && serial_->isOpen())
            {
                serial_->write("!G 1 0\r!G 2 0\r\n");
                RCLCPP_INFO(this->get_logger(), "Motors stopped");
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