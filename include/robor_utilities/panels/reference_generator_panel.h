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

#include <stdio.h>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_srvs/Empty.h>

#include <QFrame>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QComboBox>

namespace robor_utilities
{

class ReferenceGeneratorPanel : public rviz::Panel
{
Q_OBJECT
public:
  ReferenceGeneratorPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

private Q_SLOTS:
  void processInputs();
  void stop();
  void pause();
  void start();
  void activateTrajectoryParams(int index);

private:
  void verifyInputs();
  void setParams();
  void getParams();
  void evaluateParams();
  void notifyParamsUpdate();

private:
  QCheckBox* activate_checkbox_;
  QCheckBox* continuous_angle_checkbox_;
  QCheckBox* local_frame_checkbox_;

  QPushButton* stop_button_;
  QPushButton* pause_button_;
  QPushButton* play_button_;
  QPushButton* set_button_;

  QLineEdit* x_input_;
  QLineEdit* y_input_;
  QLineEdit* theta_input_;
  QLineEdit* v_input_;
  QLineEdit* T_input_;
  QLineEdit* r_x_input_;
  QLineEdit* r_y_input_;
  QLineEdit* n_x_input_;
  QLineEdit* n_y_input_;

  QComboBox* trajectories_list_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::ServiceClient params_cli_;

  // Parameters
  bool p_active_;
  bool p_continuous_angle_;
  bool p_use_local_frame_;

  bool p_paused_;
  bool p_stopped_;

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
};

} // namespace youbot_utilities
