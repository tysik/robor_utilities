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

#include "robor_utilities/simulator.h"

using namespace robor_utilities;
using namespace std;

Simulator::Simulator(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;

  timer_ = nh_.createTimer(ros::Duration(1.0), &Simulator::timerCallback, this, false, false);
  params_srv_ = nh_local_.advertiseService("params", &Simulator::updateParams, this);
  initialize();
}

bool Simulator::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  nh_local_.param<bool>("active", p_active_, false);
  nh_local_.param<bool>("continuous_angle", p_continuous_angle_, true);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  p_sampling_time_ = 1.0 / p_loop_rate_;

  nh_local_.param<double>("init_x", p_init_pose_.x, 0.0);
  nh_local_.param<double>("init_y", p_init_pose_.y, 0.0);
  nh_local_.param<double>("init_theta", p_init_pose_.theta, 0.0);

  pose_ = p_init_pose_;

  nh_local_.param<double>("time_constant", p_time_constant_, 0.0);
  nh_local_.param<double>("time_delay", p_time_delay_, 0.0);

  if (p_time_delay_ > p_sampling_time_) {
    lagged_pose_.assign(static_cast<int>(p_time_delay_ / p_sampling_time_), pose_);
    lagged_velocity_.assign(static_cast<int>(p_time_delay_ / p_sampling_time_), velocity_);
  }
  else  {
    lagged_pose_.assign(1, geometry_msgs::Pose2D());
    lagged_velocity_.assign(1, geometry_msgs::Twist());
  }

  nh_local_.param<string>("parent_frame_id", p_parent_frame_id_, "odom");
  nh_local_.param<string>("child_frame_id", p_child_frame_id_, "simulator");

  timer_.setPeriod(ros::Duration(p_sampling_time_), false);

  if (p_active_) {
    controls_sub_ = nh_.subscribe("controls", 5, &Simulator::controlsCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("simulator_state", 5);

    timer_.start();
  }
  else {
    controls_ = geometry_msgs::Twist();
    lagged_pose_.assign(lagged_pose_.size(), pose_);
    lagged_velocity_.assign(lagged_velocity_.size(), velocity_);

    controls_sub_.shutdown();
    odom_pub_.shutdown();

    timer_.stop();
  }

  return true;
}

void Simulator::timerCallback(const ros::TimerEvent& e) {
  computeVelocity();
  computePose();
  publishAll();
}

void Simulator::controlsCallback(const geometry_msgs::Twist::ConstPtr controls_msg) {
  controls_ = *controls_msg;
}

void Simulator::computeVelocity() {
  velocity_.linear.x  += p_sampling_time_ / (p_sampling_time_ + p_time_constant_) * (controls_.linear.x - velocity_.linear.x);
  velocity_.linear.y  += p_sampling_time_ / (p_sampling_time_ + p_time_constant_) * (controls_.linear.y - velocity_.linear.y);
  velocity_.angular.z += p_sampling_time_ / (p_sampling_time_ + p_time_constant_) * (controls_.angular.z - velocity_.angular.z);

  lagged_velocity_.push_back(velocity_);
}

void Simulator::computePose() {
  pose_.x += p_sampling_time_ * (velocity_.linear.x * cos(pose_.theta) - velocity_.linear.y * sin(pose_.theta));
  pose_.y += p_sampling_time_ * (velocity_.linear.x * sin(pose_.theta) + velocity_.linear.y * cos(pose_.theta));
  pose_.theta += p_sampling_time_ * velocity_.angular.z;

  if (!p_continuous_angle_)
    pose_.theta = atan2(sin(pose_.theta), cos(pose_.theta));

  lagged_pose_.push_back(pose_);
}

void Simulator::publishAll() {
  ros::Time now = ros::Time::now();

  geometry_msgs::Twist velocity = lagged_velocity_.front();
  geometry_msgs::Pose2D pose = lagged_pose_.front();
  geometry_msgs::Quaternion rotation = tf::createQuaternionMsgFromYaw(pose.theta);

  geometry_msgs::TransformStamped odom_tf;

  odom_tf.header.stamp = now;
  odom_tf.header.frame_id = p_parent_frame_id_;
  odom_tf.child_frame_id = p_child_frame_id_;

  odom_tf.transform.translation.x = pose.x;
  odom_tf.transform.translation.y = pose.y;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation = rotation;

  nav_msgs::OdometryPtr odom_msg(new nav_msgs::Odometry);

  odom_msg->header.stamp = now;
  odom_msg->header.frame_id = p_parent_frame_id_;
  odom_msg->child_frame_id = p_child_frame_id_;

  odom_msg->pose.pose.position.x = pose.x;
  odom_msg->pose.pose.position.y = pose.y;
  odom_msg->pose.pose.position.z = 0.0;
  odom_msg->pose.pose.orientation = rotation;

  odom_msg->twist.twist.linear.x = velocity.linear.x;
  odom_msg->twist.twist.linear.y = velocity.linear.y;
  odom_msg->twist.twist.angular.z = velocity.angular.z;

  tf_bc_.sendTransform(odom_tf);
  odom_pub_.publish(odom_msg);
}
