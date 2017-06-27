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

#include "robor_utilities/panels/state_estimator_panel.h"

using namespace robor_utilities;

StateEstimatorPanel::StateEstimatorPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), nh_local_("state_estimator") {
  params_cli_ = nh_local_.serviceClient<std_srvs::Empty>("params");
  getParams();

  activate_checkbox_          = new QCheckBox("On/Off");
  continuous_angle_checkbox_  = new QCheckBox("Continuous angle");

  QFrame* lines[2];
  for (auto& line : lines) {
    line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
  }

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addWidget(lines[0]);
  layout->addWidget(continuous_angle_checkbox_);
  layout->addWidget(lines[1]);
  layout->setAlignment(layout, Qt::AlignCenter);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(continuous_angle_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));

  evaluateParams();
}

void StateEstimatorPanel::processInputs() {
  verifyInputs();
  setParams();
  evaluateParams();
  notifyParamsUpdate();
}

void StateEstimatorPanel::verifyInputs() {
  p_active_ = activate_checkbox_->isChecked();
  p_continuous_angle_ = continuous_angle_checkbox_->isChecked();
}

void StateEstimatorPanel::setParams() {
  nh_local_.setParam("active", p_active_);
  nh_local_.setParam("continuous_angle", p_continuous_angle_);
}

void StateEstimatorPanel::getParams() {
  nh_local_.param<bool>("active", p_active_, false);
  nh_local_.param<bool>("continuous_angle", p_continuous_angle_, true);
}

void StateEstimatorPanel::evaluateParams() {
  activate_checkbox_->setChecked(p_active_);
  continuous_angle_checkbox_->setEnabled(p_active_);
  continuous_angle_checkbox_->setChecked(p_continuous_angle_);
}

void StateEstimatorPanel::notifyParamsUpdate() {
  std_srvs::Empty empty;
  if (!params_cli_.call(empty)) {
    p_active_ = false;
    setParams();
    evaluateParams();
  }
}

void StateEstimatorPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void StateEstimatorPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robor_utilities::StateEstimatorPanel, rviz::Panel)
