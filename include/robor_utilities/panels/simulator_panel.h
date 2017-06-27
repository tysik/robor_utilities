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
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>

namespace robor_utilities
{

class SimulatorPanel : public rviz::Panel
{
Q_OBJECT
public:
  SimulatorPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

private Q_SLOTS:
  void processInputs();

private:
  void verifyInputs();
  void setParams();
  void getParams();
  void evaluateParams();
  void notifyParamsUpdate();

private:
  QCheckBox* activate_checkbox_;
  QCheckBox* continuous_angle_checkbox_;

  QPushButton* set_button_;
  QLineEdit* t_f_input_;
  QLineEdit* t_o_input_;

  QLineEdit* init_x_input_;
  QLineEdit* init_y_input_;
  QLineEdit* init_theta_input_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::ServiceClient params_cli_;

  // Parameters
  bool p_active_;
  bool p_continuous_angle_;

  double p_sampling_time_;
  double p_time_constant_;
  double p_time_delay_;

  double p_init_x_;
  double p_init_y_;
  double p_init_theta_;
};

} // namespace youbot_utilities
