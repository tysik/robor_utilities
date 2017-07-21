/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>

namespace robor_utilities
{

class Simulator
{
public:
  Simulator(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~Simulator();

private:
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void timerCallback(const ros::TimerEvent& e);
  void controlsCallback(const geometry_msgs::Twist::ConstPtr controls_msg);

  void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }

  void computeVelocity();
  void computePose();
  void publishAll();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::ServiceServer params_srv_;
  ros::Timer timer_;

  ros::Subscriber controls_sub_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_bc_;

  geometry_msgs::Pose2D pose_;
  geometry_msgs::Twist velocity_;
  geometry_msgs::Twist controls_;

  boost::circular_buffer<geometry_msgs::Pose2D> lagged_pose_;
  boost::circular_buffer<geometry_msgs::Twist> lagged_velocity_;

  // Parameters
  bool p_active_;
  bool p_continuous_angle_;

  double p_loop_rate_;
  double p_sampling_time_;
  double p_time_constant_;
  double p_time_delay_;

  std::string p_parent_frame_id_;
  std::string p_child_frame_id_;

  geometry_msgs::Pose2D p_init_pose_;
};

} // namespace robor_utilities
