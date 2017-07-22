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

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>

#include "robor_utilities/utilities/trajectories.h"

namespace robor_utilities
{

class ReferenceGenerator
{
public:
   ReferenceGenerator(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~ReferenceGenerator();

private:   
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void timerCallback(const ros::TimerEvent& e);

  void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }

  void start();
  void stop();
  void pause();
  void update(double dt);
  void publishAll();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::ServiceServer params_srv_;

  ros::Timer timer_;

  ros::Publisher ref_twist_pub_;

  tf::TransformBroadcaster tf_bc_;

  double time_;
  Trajectory* trajectory_;

  geometry_msgs::Pose2D pose_;
  geometry_msgs::Twist velocity_;

  // Parameters
  bool p_active_;
  bool p_continuous_angle_;
  bool p_use_local_frame_;

  bool p_paused_;
  bool p_stopped_;

  double p_loop_rate_;
  double p_sampling_time_;

  int p_trajectory_type_;
  double p_x_0_;
  double p_y_0_;
  double p_theta_0_;
  double p_v_;
  double p_T_;
  double p_r_x_;
  double p_r_y_;
  double p_n_x_;
  double p_n_y_;

  std::string p_parent_frame_id_;
  std::string p_child_frame_id_;
};

} // namespace robor_utilities
