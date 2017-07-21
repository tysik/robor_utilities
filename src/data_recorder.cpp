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

#include "robor_utilities/data_recorder.h"

using namespace robor_utilities;
using namespace std;

DataRecorder::DataRecorder(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;
  p_recording_ = false;

  timer_ = nh_.createTimer(ros::Duration(1.0), &DataRecorder::timerCallback, this, false, false);
  params_srv_ = nh_local_.advertiseService("params", &DataRecorder::updateParams, this);
  initialize();
}

DataRecorder::~DataRecorder() {
  if (p_recording_)
    stop();

  nh_local_.deleteParam("active");
  nh_local_.deleteParam("recording");
  nh_local_.deleteParam("record_pose");
  nh_local_.deleteParam("record_reference_pose");
  nh_local_.deleteParam("record_velocity");
  nh_local_.deleteParam("record_reference_velocity");
  nh_local_.deleteParam("record_controls");
  nh_local_.deleteParam("loop_rate");
}

bool DataRecorder::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  bool prev_recording = p_recording_;

  nh_local_.param<bool>("active", p_active_, false);
  nh_local_.param<bool>("recording", p_recording_, false);

  nh_local_.param<bool>("record_pose", p_record_pose_, true);
  nh_local_.param<bool>("record_reference_pose", p_record_reference_pose_, true);
  nh_local_.param<bool>("record_velocity", p_record_velocity_, true);
  nh_local_.param<bool>("record_reference_velocity", p_record_reference_velocity_, true);
  nh_local_.param<bool>("record_controls", p_record_controls_, true);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  p_sampling_time_ = 1.0 / p_loop_rate_;

  timer_.setPeriod(ros::Duration(p_sampling_time_), false);

  if (p_active_) {
    if (p_recording_ && !prev_recording)
      start();
    else if (!p_recording_ && prev_recording)
      stop();

    pose_sub_ = nh_.subscribe("pose", 5, &DataRecorder::poseCallback, this);
    ref_pose_sub_ = nh_.subscribe("reference_pose", 5, &DataRecorder::refPoseCallback, this);
    velocity_sub_ = nh_.subscribe("velocity", 5, &DataRecorder::velocityCallback, this);
    ref_velocity_sub_ = nh_.subscribe("reference_velocity", 5, &DataRecorder::refVelocityCallback, this);
    controls_sub_ = nh_.subscribe("controls", 5, &DataRecorder::controlsCallback, this);

    timer_.start();
  }
  else {
    if (p_recording_)
      stop();

    pose_sub_.shutdown();
    ref_pose_sub_.shutdown();
    velocity_sub_.shutdown();
    ref_velocity_sub_.shutdown();
    controls_sub_.shutdown();

    timer_.stop();
  }

  return true;
}

void DataRecorder::timerCallback(const ros::TimerEvent& e) {
  if (p_recording_)
    addLatestData();
}

void DataRecorder::poseCallback(const geometry_msgs::Pose2D::ConstPtr pose_msg) {
  pose_ = *pose_msg;
}

void DataRecorder::refPoseCallback(const geometry_msgs::Pose2D::ConstPtr ref_pose_msg) {
  ref_pose_ = *ref_pose_msg;
}

void DataRecorder::velocityCallback(const geometry_msgs::Twist::ConstPtr velocity_msg) {
  velocity_ = *velocity_msg;
}

void DataRecorder::refVelocityCallback(const geometry_msgs::Twist::ConstPtr ref_velocity_msg) {
  ref_velocity_ = *ref_velocity_msg;
}

void DataRecorder::controlsCallback(const geometry_msgs::Twist::ConstPtr controls_msg) {
  controls_ = *controls_msg;
}

void DataRecorder::start() {
  start_mark_ = ros::Time::now();
}

void DataRecorder::stop() {
  emitTxtFile();

  t_.clear();
  pose_list_.clear();
  ref_pose_list_.clear();
  velocity_list_.clear();
  ref_velocity_list_.clear();
  controls_list_.clear();
}

void DataRecorder::addLatestData() {
  double t = (ros::Time::now() - start_mark_).toSec();
  t_.push_back(t);

  if (p_record_pose_)
    pose_list_.push_back(pose_);
  if (p_record_reference_pose_)
    ref_pose_list_.push_back(ref_pose_);
  if (p_record_velocity_)
    velocity_list_.push_back(velocity_);
  if (p_record_reference_velocity_)
    ref_velocity_list_.push_back(ref_velocity_);
  if (p_record_controls_)
    controls_list_.push_back(controls_);
}

void DataRecorder::emitTxtFile() {  
  time_t now = time(NULL);
  char the_date[30];

  if (now != -1)
    strftime(the_date, 30, "%Y_%m_%d_%H_%M_%S", gmtime(&now));

  string home_path = getenv("HOME");
  string folder_name = home_path + "/YouBot/records/";
  string file_name = folder_name + "record_" + string(the_date) + ".txt";

  boost::filesystem::create_directories(folder_name);
  std::ofstream file(file_name);

  // Create header line
  file << "ROS time at start: " << ros::Time::now() << "\n";
  file << "n \t t";
  if (p_record_pose_)
    file << " \t x \t y \t theta";
  if (p_record_reference_pose_)
    file << " \t x_r \t y_r \t theta_r";
  if (p_record_velocity_)
    file << " \t x_p \t y_p \t theta_p";
  if (p_record_reference_velocity_)
    file << " \t x_p_r \t y_p_r \t theta_p_r";
  if (p_record_controls_)
    file << " \t u \t v \t w";
  file << "\n";

  for (int i = 0; i < t_.size(); ++i) {
    file << i+1 << "\t" << t_[i];
    if (p_record_pose_)
      file << "\t" << pose_list_[i].x << "\t" << pose_list_[i].y << "\t" << pose_list_[i].theta;
    if (p_record_reference_pose_)
      file << "\t" << ref_pose_list_[i].x << "\t" << ref_pose_list_[i].y << "\t" << ref_pose_list_[i].theta;
    if (p_record_velocity_)
      file << "\t" << velocity_list_[i].linear.x << "\t" << velocity_list_[i].linear.y << "\t" << velocity_list_[i].angular.z;
    if (p_record_reference_velocity_)
      file << "\t" << ref_velocity_list_[i].linear.x << "\t" << ref_velocity_list_[i].linear.y << "\t" << ref_velocity_list_[i].angular.z;
    if (p_record_controls_)
      file << "\t" << controls_list_[i].linear.x << "\t" << controls_list_[i].linear.y << "\t" << controls_list_[i].angular.z;
    file << "\n";
  }

  file.close();
}
