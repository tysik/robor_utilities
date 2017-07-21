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

#include "robor_utilities/youbot_visualizer.h"

using namespace robor_utilities;
using namespace std;

YoubotVisualizer::YoubotVisualizer(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;

  timer_ = nh_.createTimer(ros::Duration(1.0), &YoubotVisualizer::timerCallback, this, false, false);
  params_srv_ = nh_local_.advertiseService("params", &YoubotVisualizer::updateParams, this);
  initialize();
}

YoubotVisualizer::~YoubotVisualizer() {
  nh_local_.deleteParam("active");
  nh_local_.deleteParam("emit_yb_marker");
  nh_local_.deleteParam("loop_rate");
  nh_local_.deleteParam("frame_id");
}

bool YoubotVisualizer::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("emit_yb_marker", p_emit_yb_marker_, false);
  nh_local_.param<double>("loop_rate", p_loop_rate_, 25.0);
  nh_local_.param<string>("frame_id", p_frame_id_, "robot");

  timer_.setPeriod(ros::Duration(1.0 / p_loop_rate_), false);

  if (p_active_ != prev_active) {
    if (p_active_) {
      yb_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("yb_marker", 10);
      timer_.start();
    }
    else {
      yb_marker_pub_.shutdown();
      timer_.stop();
    }
  }

  return true;
}

void YoubotVisualizer::timerCallback(const ros::TimerEvent& e) {
  if (p_emit_yb_marker_)
    publishMarkers();
}

void YoubotVisualizer::publishMarkers() {
  visualization_msgs::MarkerArrayPtr yb_markers(new visualization_msgs::MarkerArray);
  visualization_msgs::Marker marker;

  marker.header.frame_id = p_frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.lifetime = ros::Duration();
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;

  // Base
  marker.ns = "base";
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = "package://youbot_description/meshes/youbot_base/base.dae";

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = -0.5;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color.a = 1.0;
  marker.color.r = 0.949;
  marker.color.g = 0.325;
  marker.color.b = 0.137;

  yb_markers->markers.push_back(marker);

  // Plate
  marker.ns = "plate";
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = "package://youbot_description/meshes/youbot_plate/plate.dae";

  marker.pose.position.x = -0.16;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.04 - 0.5;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color.a = 1.0;
  marker.color.r = 0.1;
  marker.color.g = 0.1;
  marker.color.b = 0.1;

  yb_markers->markers.push_back(marker);

  // Front Hokuyo
  marker.ns = "front_hokuyo";
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = "package://youbot_description/meshes/sensors/hokuyo.dae";

  marker.pose.position.x = 0.22565;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.1 - 0.5;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color.a = 1.0;
  marker.color.r = 0.2;
  marker.color.g = 0.2;
  marker.color.b = 0.2;

  yb_markers->markers.push_back(marker);

  // Rear Hokuyo
  marker.ns = "rear_hokuyo";
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = "package://youbot_description/meshes/sensors/hokuyo.dae";

  marker.pose.position.x = -0.25832;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.1 - 0.5;

  marker.pose.orientation = tf::createQuaternionMsgFromYaw(3.14159265);

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color.a = 1.0;
  marker.color.r = 0.2;
  marker.color.g = 0.2;
  marker.color.b = 0.2;

  yb_markers->markers.push_back(marker);

  yb_marker_pub_.publish(yb_markers);
}
