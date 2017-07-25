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

#include "robor_utilities/optitrack_drift.h"

using namespace robor_utilities;
using namespace std;

OptitrackDrift::OptitrackDrift(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;

  timer_ = nh_.createTimer(ros::Duration(1.0), &OptitrackDrift::timerCallback, this, false, false);
  params_srv_ = nh_local_.advertiseService("params", &OptitrackDrift::updateParams, this);
  initialize();
}

OptitrackDrift::~OptitrackDrift() {
  nh_local_.deleteParam("active");
  nh_local_.deleteParam("loop_rate");
  nh_local_.deleteParam("map_frame_id");
  nh_local_.deleteParam("odom_frame_id");
  nh_local_.deleteParam("robot_frame_id");
  nh_local_.deleteParam("optitrack_frame_id");
}

bool OptitrackDrift::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  nh_local_.param<bool>("active", p_active_, false);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 100.0);
  p_sampling_time_ = 1.0 / p_loop_rate_;

  nh_local_.param<string>("map_frame_id", p_map_frame_id_, "map");
  nh_local_.param<string>("odom_frame_id", p_odom_frame_id_, "odom");
  nh_local_.param<string>("robot_frame_id", p_robot_frame_id_, "robot");
  nh_local_.param<string>("optitrack_frame_id", p_optitrack_frame_id_, "optitrack");

  timer_.setPeriod(ros::Duration(p_sampling_time_), false);

  if (p_active_)
    timer_.start();
  else
    timer_.stop();

  return true;
}

void OptitrackDrift::timerCallback(const ros::TimerEvent& e) {
  ros::Time now = ros::Time::now();

  // We assume that optitrack and robot are the same frames (we have two sources of them)
  tf::StampedTransform opti_in_map;
  tf::StampedTransform robot_in_odom;

  try {
    tf_ls_.waitForTransform(p_map_frame_id_, p_optitrack_frame_id_, now, ros::Duration(1.0));
    tf_ls_.lookupTransform(p_map_frame_id_, p_optitrack_frame_id_, now, opti_in_map);
  }
  catch (tf::TransformException& e) {
    opti_in_map.setIdentity();
  }

  try {
    tf_ls_.waitForTransform(p_odom_frame_id_, p_robot_frame_id_, now, ros::Duration(1.0));
    tf_ls_.lookupTransform(p_odom_frame_id_, p_robot_frame_id_, now, robot_in_odom);
  }
  catch (tf::TransformException& e) {
    robot_in_odom.setIdentity();
  }

  tf::Transform odom_in_map = opti_in_map * robot_in_odom.inverse();
  tf::StampedTransform odom_in_map_s(odom_in_map, now, p_map_frame_id_, p_odom_frame_id_);

  geometry_msgs::TransformStamped odom_in_map_msg;
  tf::transformStampedTFToMsg(odom_in_map_s, odom_in_map_msg);

  tf_bc_.sendTransform(odom_in_map_msg);
}
