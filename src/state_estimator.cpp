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

#include "robor_utilities/state_estimator.h"

using namespace robor_utilities;
using namespace arma;
using namespace std;

StateEstimator::StateEstimator(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local), angular_kf_(1, 1, 2), linear_kf_(2, 2, 4) {
  p_active_ = false;

  timer_ = nh_.createTimer(ros::Duration(1.0), &StateEstimator::timerCallback, this, false, false);
  params_srv_ = nh_local_.advertiseService("params", &StateEstimator::updateParams, this);

  initialize();
}

bool StateEstimator::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, false);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  p_sampling_time_ = 1.0 / p_loop_rate_;

  nh_local_.param<double>("angular_var", p_angular_var_, 0.1);
  nh_local_.param<double>("angular_rate_var", p_angular_rate_var_, 0.1);
  nh_local_.param<double>("linear_var", p_linear_var_, 0.1);
  nh_local_.param<double>("linear_rate_var", p_linear_rate_var_, 0.1);

  nh_local_.param<string>("parent_frame_id", p_parent_frame_id_, "odom");
  nh_local_.param<string>("child_frame_id", p_child_frame_id_, "robot");

  timer_.setPeriod(ros::Duration(p_sampling_time_), false);

  if (p_active_ != prev_active) {
    if (p_active_) {
      controls_sub_ = nh_.subscribe("controls", 10, &StateEstimator::controlsCallback, this);
      odom_sub_ = nh_.subscribe("odom", 10, &StateEstimator::odomCallback, this);
      odom_pub_ = nh_.advertise<nav_msgs::Odometry>("robot_state", 10);

      timer_.start();
    }
    else {
      controls_sub_.shutdown();
      odom_sub_.shutdown();
      odom_pub_.shutdown();

      timer_.stop();
    }
  }

//  try {
//    tf::StampedTransform transform;
//    ros::Time now = ros::Time::now();
//    tf_ls_.waitForTransform(p_parent_frame_id_, p_child_frame_id_, now, ros::Duration(10.0));
//    tf_ls_.lookupTransform(p_parent_frame_id_, p_child_frame_id_, now, transform);

//    pose_.x = transform.getOrigin().getX();
//    pose_.y = transform.getOrigin().getY();
//    pose_.theta = tf::getYaw(transform.getRotation());
//  }
//  catch (tf::TransformException ex) {
//    ROS_WARN_STREAM(ex.what());
//    return false;
//  }

  // Initialize angular KF
  angular_kf_.A << 1.0 << p_sampling_time_ << endr
                << 0.0 << 0.0 << endr;

  angular_kf_.B << 0.0 << endr
                << 1.0 << endr;

  angular_kf_.C << 1.0 << 0.0 << endr;

  angular_kf_.Q << p_angular_var_ << 0.0                 << endr
                << 0.0            << p_angular_rate_var_ << endr;

  double cos_phi = cos(pose_.theta);
  double sin_phi = sin(pose_.theta);

  // Initialize linear KF
  linear_kf_.A << 1.0 << p_sampling_time_ * cos_phi << 0.0 << -p_sampling_time_ * sin_phi << endr
               << 0.0 << 0.0                        << 0.0 << 0.0                         << endr
               << 0.0 << p_sampling_time_ * sin_phi << 1.0 << p_sampling_time_ * cos_phi  << endr
               << 0.0 << 0.0                        << 0.0 << 0.0                         << endr;

  linear_kf_.B << 0.0 << 0.0 << endr
               << 1.0 << 0.0 << endr
               << 0.0 << 0.0 << endr
               << 0.0 << 1.0 << endr;

  linear_kf_.C << 1.0 << 0.0 << 0.0 << 0.0 << endr
               << 0.0 << 0.0 << 1.0 << 0.0 << endr;

  linear_kf_.Q(0, 0) = linear_kf_.Q(2, 2) = p_linear_var_;
  linear_kf_.Q(1, 1) = linear_kf_.Q(3, 3) = p_linear_rate_var_;

  return true;
}

void StateEstimator::timerCallback(const ros::TimerEvent& e) {
  static geometry_msgs::Pose2D pose;
  double prev_theta = pose.theta;

  ros::Time now = ros::Time::now();

  // Update pose measurement
  try {
    tf::StampedTransform transform;
    tf_ls_.waitForTransform(p_parent_frame_id_, p_child_frame_id_, now, ros::Duration(10.0));
    tf_ls_.lookupTransform(p_parent_frame_id_, p_child_frame_id_, now, transform);

    pose.x = transform.getOrigin().getX();
    pose.y = transform.getOrigin().getY();
    pose.theta = tf::getYaw(transform.getRotation());
  }
  catch (tf::TransformException ex) {
    ROS_WARN_STREAM(ex.what());
    return;
  }

  // Make angle continuous
  double theta_diff = atan2(sin(pose.theta), cos(pose.theta)) - atan2(sin(prev_theta), cos(prev_theta));

  if (theta_diff < -M_PI)
    pose.theta = prev_theta + theta_diff + 2.0 * M_PI;
  else if (theta_diff > M_PI)
    pose.theta = prev_theta + theta_diff - 2.0 * M_PI;
  else
    pose.theta = prev_theta + theta_diff;

  // Update angular KF
  angular_kf_.u = odom_.twist.twist.angular.z;
  angular_kf_.y = pose.theta;
  angular_kf_.updateState();

  pose_.theta = angular_kf_.q_est(0);
  velocity_.angular.z = angular_kf_.q_est(1);

  // Update linear KF
  double cos_phi = cos(pose_.theta);
  double sin_phi = sin(pose_.theta);

  linear_kf_.u << odom_.twist.twist.linear.x << odom_.twist.twist.linear.y;
  linear_kf_.y << pose.x << pose.y;
  linear_kf_.A << 1.0 << p_sampling_time_ * cos_phi << 0.0 << -p_sampling_time_ * sin_phi << endr
               << 0.0 << 0.0                        << 0.0 << 0.0                         << endr
               << 0.0 << p_sampling_time_ * sin_phi << 1.0 << p_sampling_time_ * cos_phi  << endr
               << 0.0 << 0.0                        << 0.0 << 0.0                         << endr;
  linear_kf_.updateState();

  pose_.x = linear_kf_.q_est(0);
  pose_.y = linear_kf_.q_est(2);

  velocity_.linear.x = linear_kf_.q_est(1);
  velocity_.linear.y = linear_kf_.q_est(3);

  geometry_msgs::Quaternion rotation = tf::createQuaternionMsgFromYaw(pose_.theta);

//  geometry_msgs::TransformStamped odom_tf;

//  odom_tf.header.stamp = now;
//  odom_tf.header.frame_id = p_parent_frame_id_;
//  odom_tf.child_frame_id = p_child_frame_id_;

//  odom_tf.transform.translation.x = pose_.x;
//  odom_tf.transform.translation.y = pose_.y;
//  odom_tf.transform.translation.z = 0.0;
//  odom_tf.transform.rotation = rotation;

//  tf_bc_.sendTransform(odom_tf);

  nav_msgs::OdometryPtr odom_msg(new nav_msgs::Odometry);

  odom_msg->header.stamp = odom_.header.stamp;
  odom_msg->header.frame_id = p_parent_frame_id_;
  odom_msg->child_frame_id = p_child_frame_id_;

  odom_msg->pose.pose.position.x = pose_.x;
  odom_msg->pose.pose.position.y = pose_.y;
  odom_msg->pose.pose.position.z = 0.0;
  odom_msg->pose.pose.orientation = rotation;

  odom_msg->twist.twist = velocity_;

  odom_pub_.publish(odom_msg);
}

void StateEstimator::controlsCallback(const geometry_msgs::Twist::ConstPtr controls_msg) {
  controls_ = *controls_msg;
}

void StateEstimator::odomCallback(const nav_msgs::Odometry::ConstPtr odom_msg) {
  odom_ = *odom_msg;
}

//void StateEstimator::optiPoseCallback(const geometry_msgs::Pose2D::ConstPtr opti_pose_msg) {
//  double prev_opti_theta = opti_pose_.theta;
//  double prev_opti_theta_aux = atan2(sin(opti_pose_.theta), cos(opti_pose_.theta));

//  opti_pose_ = *opti_pose_msg;

//  if (p_continuous_angle_) {
//    double new_theta_aux = atan2(sin(opti_pose_.theta), cos(opti_pose_.theta));
//    double theta_diff = new_theta_aux - prev_opti_theta_aux;

//    if (theta_diff < -M_PI)
//      opti_pose_.theta = prev_opti_theta + theta_diff + 2.0 * M_PI;
//    else if (theta_diff > M_PI)
//      opti_pose_.theta = prev_opti_theta + theta_diff - 2.0 * M_PI;
//    else
//      opti_pose_.theta = prev_opti_theta + theta_diff;
//  }
//}
