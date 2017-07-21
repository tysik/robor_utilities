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

#include "robor_utilities/reference_generator.h"

using namespace robor_utilities;
using namespace std;

ReferenceGenerator::ReferenceGenerator(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;

  timer_ = nh_.createTimer(ros::Duration(1.0), &ReferenceGenerator::timerCallback, this, false, false);
  params_srv_ = nh_local_.advertiseService("params", &ReferenceGenerator::updateParams, this);
  initialize();
}

ReferenceGenerator::~ReferenceGenerator() {
  delete trajectory_;

  nh_local_.deleteParam("active");
  nh_local_.deleteParam("continuous_angle");
  nh_local_.deleteParam("trajectory_paused");
  nh_local_.deleteParam("trajectory_stopped");

  nh_local_.deleteParam("loop_rate");

  nh_local_.deleteParam("parent_frame_id");
  nh_local_.deleteParam("child_frame_id");

  nh_local_.deleteParam("trajectory_type");

  nh_local_.deleteParam("initial_x");
  nh_local_.deleteParam("initial_y");
  nh_local_.deleteParam("initial_theta");

  nh_local_.deleteParam("linear_velocity");
  nh_local_.deleteParam("harmonic_period");
  nh_local_.deleteParam("harmonic_radius_x");
  nh_local_.deleteParam("harmonic_radius_y");
  nh_local_.deleteParam("harmonic_multiplier_x");
  nh_local_.deleteParam("harmonic_multiplier_y");
}

bool ReferenceGenerator::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, false);
  nh_local_.param<bool>("continuous_angle", p_continuous_angle_, true);
  nh_local_.param<bool>("trajectory_paused", p_paused_, true);
  nh_local_.param<bool>("trajectory_stopped", p_stopped_, false);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  p_sampling_time_ = 1.0 / p_loop_rate_;

  nh_local_.param<string>("parent_frame_id", p_parent_frame_id_, "odom");
  nh_local_.param<string>("child_frame_id", p_child_frame_id_, "reference");

  nh_local_.param<int>("trajectory_type", p_trajectory_type_, 0);

  nh_local_.param<double>("initial_x", p_x_0_, 0.0);
  nh_local_.param<double>("initial_y", p_y_0_, 0.0);
  nh_local_.param<double>("initial_theta", p_theta_0_, 0.0);

  nh_local_.param<double>("linear_velocity", p_v_, 0.1);
  nh_local_.param<double>("harmonic_period", p_T_, 5.0);
  nh_local_.param<double>("harmonic_radius_x", p_r_x_, 1.0);
  nh_local_.param<double>("harmonic_radius_y", p_r_y_, 1.0);
  nh_local_.param<double>("harmonic_multiplier_x", p_n_x_, 1.0);
  nh_local_.param<double>("harmonic_multiplier_y", p_n_y_, 1.0);

  switch (p_trajectory_type_) {
    case 0:
      trajectory_ = new PointTrajectory(p_x_0_, p_y_0_, p_theta_0_); break;
    case 1:
      trajectory_ = new LinearTrajectory(p_x_0_, p_y_0_, p_theta_0_, p_v_); break;
    case 2:
      trajectory_ = new HarmonicTrajectory(p_x_0_, p_y_0_, p_T_, p_r_x_, p_r_y_, p_n_x_, p_n_y_); break;
    case 3:
      trajectory_ = new LemniscateTrajectory(p_x_0_, p_y_0_, p_T_, p_r_x_, p_r_y_, p_n_x_, p_n_y_); break;
    default:
      trajectory_ = new PointTrajectory(0.0, 0.0, 0.0); break;
  }

  timer_.setPeriod(ros::Duration(p_sampling_time_), false);

  if (p_active_) {
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("reference_state", 5);

    if (!p_paused_ && !p_stopped_)
      start();
    else if (p_paused_ && !p_stopped_)
      pause();
    else
      stop();

    timer_.start();
  }
  else {
    if (prev_active)
      pause();

    odom_pub_.shutdown();

    timer_.stop();
  }

  return true;
}

void ReferenceGenerator::timerCallback(const ros::TimerEvent& e) {
  if (!p_paused_ && !p_stopped_) {
    update(p_sampling_time_);
    publishAll();
  }
}

void ReferenceGenerator::start() {
  p_paused_ = false;
}

void ReferenceGenerator::stop() {
  p_paused_ = true;
  time_ = 0.0;
  update(0.0);

  velocity_ = geometry_msgs::Twist();
  publishAll();
}

void ReferenceGenerator::pause() {
  p_paused_ = true;

  velocity_ = geometry_msgs::Twist();
  publishAll();
}

void ReferenceGenerator::update(double dt) {
  double prev_theta = pose_.theta;
  double prev_theta_aux = atan2(sin(pose_.theta), cos(pose_.theta));

  time_ += dt;
  pose_ = trajectory_->calculatePose(time_);
  velocity_ = trajectory_->calculateVelocity(time_);

  if (p_continuous_angle_ ) {
    double new_theta_aux = atan2(sin(pose_.theta), cos(pose_.theta));
    double theta_diff = new_theta_aux - prev_theta_aux;

    if (theta_diff < -M_PI)
      pose_.theta = prev_theta + theta_diff + 2.0 * M_PI;
    else if (theta_diff > M_PI)
      pose_.theta = prev_theta + theta_diff - 2.0 * M_PI;
    else
      pose_.theta = prev_theta + theta_diff;
  }
}

void ReferenceGenerator::publishAll() {
  ros::Time now = ros::Time::now();

  geometry_msgs::Quaternion rotation = tf::createQuaternionMsgFromYaw(pose_.theta);

  geometry_msgs::TransformStamped odom_tf;

  odom_tf.header.stamp = now;
  odom_tf.header.frame_id = p_parent_frame_id_;
  odom_tf.child_frame_id = p_child_frame_id_;

  odom_tf.transform.translation.x = pose_.x;
  odom_tf.transform.translation.y = pose_.y;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation = rotation;

  nav_msgs::OdometryPtr odom_msg(new nav_msgs::Odometry);

  odom_msg->header.stamp = now;
  odom_msg->header.frame_id = p_parent_frame_id_;
  odom_msg->child_frame_id = p_child_frame_id_;

  odom_msg->pose.pose.position.x = pose_.x;
  odom_msg->pose.pose.position.y = pose_.y;
  odom_msg->pose.pose.position.z = 0.0;
  odom_msg->pose.pose.orientation = rotation;

  odom_msg->twist.twist.linear.x = velocity_.linear.x;
  odom_msg->twist.twist.linear.y = velocity_.linear.y;
  odom_msg->twist.twist.angular.z = velocity_.angular.z;

  tf_bc_.sendTransform(odom_tf);
  odom_pub_.publish(odom_msg);
}
