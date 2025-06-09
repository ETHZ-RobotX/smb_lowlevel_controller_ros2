#include <memory>
#include <string>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class SpeedControlNode : public rclcpp::Node
{
    public:
        SpeedControlNode() : Node("speed_control_node"), port_("/dev/ttyUSB0"), baudrate_(115200), motor_1_channel_(1), motor_2_channel_(2), last_command_time_(this->now())
        {
            this->declare_parameter<std::string>("port", port_);                    // Parameter for the serial port
            this->declare_parameter<int>("baudrate", baudrate_);                    // Parameter for the baudrate   
            this->declare_parameter<int>("motor_1_channel", motor_1_channel_);      // Parameter for the motor 1 channel
            this->declare_parameter<int>("motor_2_channel", motor_2_channel_);      // Parameter for the motor 2 channel
            this->declare_parameter<int>("send_cmd", 1);                            // Parameter for the send command flag: 
                                                                                    // 0 - Teleop directly from RC to motor controller 
                                                                                    // 1 - send commands from this node to the motor controller

            // Connect to the motor controller
            connect();
            
            send_cmd_flag_ = this->get_parameter("send_cmd").as_int();
            RCLCPP_INFO(this->get_logger(), "Serial control flag: %d", send_cmd_flag_);
            
            // Create a publisher - publishes the motor speeds in RPM
            // wheel_speed_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_speed", 10);
            
            // Create a publisher - publishes the RC input as a Twist message
            rc_input_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("rc_vel", 10);
            
            // Create a publisher - publishes a dummy message - to check the frequency of the node
            dummy_publisher_ = this->create_publisher<std_msgs::msg::Int32>("dummy", 10);
            
            // Creating a subscriber - to receive the target speed
            wheel_speed_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("wheel_joint_commands", 10, std::bind(&SpeedControlNode::set_speed, this, std::placeholders::_1));

            //Creating a publisher - to publish the wheel positions
            wheel_pos_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            
            // Creating timer - for publishing topics
            // publish_timer_ = this->create_wall_timer(10ms, std::bind(&SpeedControlNode::publish_info, this));
            
            // Creating timer - to read motor info (speed + rc inputs) from the motor controller
            read_timer_ = this->create_wall_timer(5ms, std::bind(&SpeedControlNode::read_info, this));
            
            // Creating timer - to flush the serial port input and output buffers periodically
            flush_timer_ = this->create_wall_timer(100ms, std::bind(&SpeedControlNode::flush, this));
             
        }

        ~SpeedControlNode()
        {
            // Stop motors and disconnect from the controller
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
        
        int speed_counter_ = 0; // Counter for the speed messages

        std::string port_;
        std::unique_ptr<serial::Serial> serial_;
        
        //Used for parsing the response
        std::string prefix_ = "D=";  
        std::string delimiter_ = ":";
        int prefix_length_ = prefix_.length();
        int count_ = 0;
        std::string serial_response_;
        
        float wheel_base_ = 0.68; // in meters - distance between the wheels

        // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_speed_publisher_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_speed_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_pos_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr rc_input_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dummy_publisher_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::TimerBase::SharedPtr flush_timer_;
        rclcpp::TimerBase::SharedPtr read_timer_;
        rclcpp::Time last_command_time_;
        double prev_pos_1_ = 0.0; // Previous position of motor 1
        double prev_pos_2_ = 0.0; // Previous position of motor 2
        double vel_1_rad_per_sec = 0.0;
        double vel_2_rad_per_sec = 0.0;

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
                    std::string speed_query = "/\"d=\",\":\"?PIC 1_?PIC 2_?C 1_?C 2_# 5\r\n"; // change to PIC (page 272 in manual)
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

        // Function to set the speed of the motors
        void set_speed(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            if(send_cmd_flag_ != 0) // If the send command flag is set to 1, then send the commands to the motor controller
            {
                
                if (check_connection())
                {   
                    // DEBUG: print put message data
                    // RCLCPP_INFO(this->get_logger(), "Received message data:  %f %f %f", msg->data[0], msg->data[1], msg->data[2]);
                    int target_speed_1 = llround(msg->data[1] * 30.0 / M_PI);
                    int target_speed_2 = -llround(msg->data[2] * 30.0 / M_PI);
                    // last_command_time_ = this->now();
                    // Santity check
                    int max_speed = 50; // Max speed in RPM
                    if (target_speed_1 > max_speed)
                    {
                        target_speed_1 = max_speed;
                    }
                    else if (target_speed_1 < -max_speed)
                    {
                        target_speed_1 = -max_speed;
                    }
                    if (target_speed_2 > max_speed)
                    {
                        target_speed_2 = max_speed;
                    }
                    else if (target_speed_2 < -max_speed)
                    {
                        target_speed_2 = -max_speed;
                    }

                    if(target_speed_1 == 0 && target_speed_2 == 0)
                    {
                        stop_motors(); // This is done to hard-stop the motors if the target speed is 0 using !G command. !S is slow due to the integrator.
                    }
                    else
                    {
                        
                        std::string speed_command = "!S 1 " + std::to_string(target_speed_1) + "_" + "!S 2 " + std::to_string(target_speed_2) + "\r\n";
                        
                        // DEBUG: print speed command
                        RCLCPP_INFO(this->get_logger(), "Sending command: %s", speed_command.c_str());
                        
                        serial_->write(speed_command);
                    }
                    
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Serial port not open");
                }
            }
        }

        // Function to read the motor info (speed + rc inputs) from the motor controller
        void read_info()
        {
            if(check_connection())
            {
                
                motor_info_struct motor_info;
                std::string response = serial_->readline(30, "\r");
                // Check if it starts with prefix
                if(response.find(prefix_) == 0)
                {
                    serial_response_ = response;
                    
                    // DEBUG: print incoming data stream
                    // RCLCPP_INFO(this->get_logger(), "Response: %s", response.c_str());

                    count_++;
                    std_msgs::msg::Int32 dummy_msg;
                    dummy_msg.data = count_;
                    dummy_publisher_->publish(dummy_msg);

                    try
                    {   
                        // Publish to all topics
                        motor_info = parse_string(serial_response_);
                        // DEBUG: print motor info
                        // RCLCPP_INFO(this->get_logger(), "Motor 1 speed: %d, Motor 2 speed: %d, Pulse 1: %d, Pulse 2: %d", motor_info.motor_1_speed, motor_info.motor_2_speed, motor_info.pulse_1, motor_info.pulse_2);
                        
                        // std_msgs::msg::Float64MultiArray wheel_speed_msg;
                        sensor_msgs::msg::JointState wheel_pos_msg;
                        geometry_msgs::msg::TwistStamped rc_input_msg;
                        // wheel_speed_msg.data = {this->now().seconds(), motor_info.motor_1_speed, -motor_info.motor_2_speed};

                        double encoder_pos_1 = static_cast<double>(motor_info.encoder_pos_1);
                        double encoder_pos_2 = static_cast<double>(motor_info.encoder_pos_2);

                        double pos_1_rad = encoder_pos_1 * (2.0 * M_PI / 960.0); // Assuming 4096 counts per revolution
                        double pos_2_rad = encoder_pos_2 * (2.0 * M_PI / 960.0); // Assuming 4096 counts per revolution

                        

                        speed_counter_ ++;
                        if (speed_counter_ > 10)
                        {
                            vel_1_rad_per_sec = (pos_1_rad - prev_pos_1_) / (this->now() - last_command_time_).seconds();
                            vel_2_rad_per_sec = (pos_2_rad - prev_pos_2_) / (this->now() - last_command_time_).seconds();

                            prev_pos_1_ = pos_1_rad;
                            prev_pos_2_ = pos_2_rad;
    
                            last_command_time_ = this->now();
                            speed_counter_ = 0;
                        }


                        wheel_pos_msg.header.stamp = this->now();
                        wheel_pos_msg.name = {"left_wheel_joint", "right_wheel_joint"};
                        wheel_pos_msg.position = {pos_1_rad, pos_2_rad};
                        wheel_pos_msg.velocity = {vel_1_rad_per_sec, vel_2_rad_per_sec};

                        rc_input_msg.header.stamp = this->now();
                        rc_input_msg.twist.linear.x = 1.0*(motor_info.pulse_1 + (-motor_info.pulse_2))/1000.0; //Max linear velocity is 5 m/s
                        rc_input_msg.twist.angular.z = 1.0*(-motor_info.pulse_1 + (-motor_info.pulse_2))/(1000.0*wheel_base_); //Max angular velocity is 3 rad/s

                        // DEBUG: print rc input
                        // RCLCPP_INFO(this->get_logger(), "RC input: %f, %f", rc_input_msg.twist.linear.x, rc_input_msg.twist.angular.z);
                        if ((std::fabs(rc_input_msg.twist.linear.x) > 0.2) || (std::fabs(rc_input_msg.twist.angular.z) > 0.2)) {
                            rc_input_publisher_->publish(rc_input_msg);
                        }
                        // wheel_speed_publisher_->publish(wheel_speed_msg);
                        wheel_pos_publisher_->publish(wheel_pos_msg);
                    }
                    catch(const std::exception& e)
                    {
                        // ERROR: Handle parsing errors
                        RCLCPP_ERROR(this->get_logger(), "Error parsing speed: %s", e.what());
                    }
                
                }
                else
                {
                    // ERROR: Handle prefix mismatch
                    // RCLCPP_ERROR(this->get_logger(), "Invalid response: %s", response.c_str());
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Serial port not open");
            }
        }
        
        // Function to publish the motor info (speed + rc inputs) to the topics
        // void publish_info()
        // {
        //     motor_info_struct motor_info;
        //     if(check_connection())
        //     {

        //         try
        //         {   
        //             // Publish to all topics
        //             motor_info = parse_string(serial_response_);
        //             // DEBUG: print motor info
        //             // RCLCPP_INFO(this->get_logger(), "Motor 1 speed: %d, Motor 2 speed: %d, Pulse 1: %d, Pulse 2: %d", motor_info.motor_1_speed, motor_info.motor_2_speed, motor_info.pulse_1, motor_info.pulse_2);
                    
        //             // std_msgs::msg::Float64MultiArray wheel_speed_msg;
        //             sensor_msgs::msg::JointState wheel_pos_msg;
        //             geometry_msgs::msg::TwistStamped rc_input_msg;
        //             // wheel_speed_msg.data = {this->now().seconds(), motor_info.motor_1_speed, -motor_info.motor_2_speed};

        //             wheel_pos_msg.header.stamp = this->now();
        //             wheel_pos_msg.name = {"left_wheel_joint", "right_wheel_joint"};
        //             wheel_pos_msg.position = {static_cast<double>(motor_info.encoder_pos_1), static_cast<double>(motor_info.encoder_pos_2)};

        //             rc_input_msg.header.stamp = this->now();
        //             rc_input_msg.twist.linear.x = 1.0*(motor_info.pulse_1 + (-motor_info.pulse_2))/1000.0; //Max linear velocity is 5 m/s
        //             rc_input_msg.twist.angular.z = 1.0*(-motor_info.pulse_1 + (-motor_info.pulse_2))/(1000.0*wheel_base_); //Max angular velocity is 3 rad/s

        //             // DEBUG: print rc input
        //             // RCLCPP_INFO(this->get_logger(), "RC input: %f, %f", rc_input_msg.twist.linear.x, rc_input_msg.twist.angular.z);
        //             rc_input_publisher_->publish(rc_input_msg);
        //             // wheel_speed_publisher_->publish(wheel_speed_msg);
        //             wheel_pos_publisher_->publish(wheel_pos_msg);
        //         }
        //         catch(const std::exception& e)
        //         {
        //             // ERROR: Handle parsing errors
        //             RCLCPP_ERROR(this->get_logger(), "Error parsing speed: %s", e.what());
        //         }
        //     }
        //     else
        //     {
        //         RCLCPP_ERROR(this->get_logger(), "Serial port not open");
        //     }
        // }

        struct motor_info_struct
        {
            int pulse_1;
            int pulse_2;
            int encoder_pos_1;
            int encoder_pos_2;
        };

        // Function to parse the incoming data stream into motor speeds and rc inputs
        motor_info_struct parse_string(std::string str)
        {
            // Parses incoming data stream into the motor speeds and rc inputs
            motor_info_struct motor_info;
            std::string token;
            token = str.substr(prefix_length_, str.find(delimiter_) - prefix_length_);

            motor_info.pulse_1 = std::stod(token);
            str.erase(0, token.length() + prefix_length_ + delimiter_.length());

            token = str.substr(0, str.find(delimiter_));
            motor_info.pulse_2 = std::stod(token);
            str.erase(0, token.length() + delimiter_.length());

            token = str.substr(0, str.find(delimiter_));
            motor_info.encoder_pos_1 = std::stod(token);
            str.erase(0, token.length() + delimiter_.length());

            motor_info.encoder_pos_2 = std::stod(str);
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
                serial_->write("!G 1 0_!G 2 0\r\n");

                // RCLCPP_INFO(this->get_logger(), "Motors stopped");
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