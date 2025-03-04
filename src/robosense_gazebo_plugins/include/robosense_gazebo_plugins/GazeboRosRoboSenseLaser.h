/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019-2020, ENSTA.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef GAZEBO_ROS_ROBOSENSE_LASER_H_
#define GAZEBO_ROS_ROBOSENSE_LASER_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <sdf/sdf.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/Sensors.hh>

#if GAZEBO_GPU_RAY
#include <gazebo/plugins/GpuRayPlugin.hh>
#else
#include <gazebo/plugins/RayPlugin.hh>
#endif

#include <mutex>
#include <thread>
#include <random>

#if GAZEBO_GPU_RAY
#define RayPlugin GpuRayPlugin
#define RaySensorPtr GpuRaySensorPtr
#endif

namespace gazebo
{
  class GazeboRosRobosenseLaser : public RayPlugin
  {
  public:
    GazeboRosRobosenseLaser();
    ~GazeboRosRobosenseLaser();

    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  private:
    void ConnectCb();
    void OnScan(const ConstLaserScanStampedPtr &_msg);

    // ROS 2 Node and Publisher
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;

    // Gazebo Sensor and Transport
    sensors::RaySensorPtr parent_ray_sensor_;
    gazebo::transport::NodePtr gazebo_node_;
    gazebo::transport::SubscriberPtr sub_;

    // Parameters
    std::string topic_name_;
    std::string frame_name_;
    std::string robot_namespace_;

    double min_intensity_;
    double min_range_;
    double max_range_;
    double gaussian_noise_;

    // Threading & Mutex
    std::mutex lock_;
    std::thread callback_laser_queue_thread_;

    // Gaussian noise generator
    static double gaussianKernel(double mu, double sigma)
    {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::normal_distribution<> d(mu, sigma);
      return d(gen);
    }
  };
} // namespace gazebo

#endif /* GAZEBO_ROS_ROBOSENSE_LASER_H_ */
