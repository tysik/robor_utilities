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

#include "robor_utilities/panels/data_recorder_panel.h"

using namespace robor_utilities;

DataRecorderPanel::DataRecorderPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), nh_local_("data_recorder") {
  params_cli_ = nh_local_.serviceClient<std_srvs::Empty>("params");
  getParams();

  activate_checkbox_        = new QCheckBox("On/Off");
  pose_checkbox_            = new QCheckBox("Pose");
  ref_pose_checkbox_        = new QCheckBox("Reference pose");
  velocity_checkbox_        = new QCheckBox("Velocity");
  ref_velocity_checkbox_    = new QCheckBox("Reference velocity");
  controls_checkbox_        = new QCheckBox("Controls");

  QString home_path = getenv("HOME");
  QString play_icon = home_path + QString("/.local/share/icons/robor/play.png");
  QString stop_icon = home_path + QString("/.local/share/icons/robor/stop.png");

  start_button_ = new QPushButton;
  start_button_->setMinimumSize(50, 50);
  start_button_->setMaximumSize(50, 50);
  start_button_->setIcon(QIcon(play_icon));
  start_button_->setIconSize(QSize(25, 25));
  start_button_->setCheckable(true);

  stop_button_ = new QPushButton;
  stop_button_->setMinimumSize(50, 50);
  stop_button_->setMaximumSize(50, 50);
  stop_button_->setIcon(QIcon(stop_icon));
  stop_button_->setIconSize(QSize(25, 25));
  stop_button_->setCheckable(true);

  QFrame* lines[4];
  for (auto& line : lines) {
    line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
  }

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QHBoxLayout* buttons_layout = new QHBoxLayout;
  buttons_layout->addItem(margin);
  buttons_layout->addWidget(stop_button_);
  buttons_layout->addWidget(start_button_);
  buttons_layout->addItem(margin);

  QGridLayout* checkbox_layout = new QGridLayout;
  checkbox_layout->addItem(margin, 0, 0, 2, 1);
  checkbox_layout->addWidget(pose_checkbox_, 0, 1);
  checkbox_layout->addWidget(velocity_checkbox_, 0, 2);
  checkbox_layout->addWidget(controls_checkbox_, 0, 3);
  checkbox_layout->addWidget(ref_pose_checkbox_, 1, 1);
  checkbox_layout->addWidget(ref_velocity_checkbox_, 1, 2);
  checkbox_layout->addItem(margin, 0, 4, 2, 1);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addWidget(lines[0]);
  layout->addLayout(buttons_layout);
  layout->addWidget(lines[1]);
  layout->addLayout(checkbox_layout);
  layout->addWidget(lines[2]);
  layout->setAlignment(layout, Qt::AlignCenter);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(pose_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(ref_pose_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(velocity_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(ref_velocity_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(controls_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));

  connect(start_button_, SIGNAL(clicked()), this, SLOT(start()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stop()));

  evaluateParams();
}

void DataRecorderPanel::processInputs() {
  verifyInputs();
  setParams();
  evaluateParams();
  notifyParamsUpdate();
}

void DataRecorderPanel::start() {
  p_recording_ = true;
  processInputs();
}

void DataRecorderPanel::stop() {
  p_recording_ = false;
  processInputs();
}

void DataRecorderPanel::verifyInputs() {
  p_active_ = activate_checkbox_->isChecked();

  p_record_pose_ = pose_checkbox_->isChecked();
  p_record_reference_pose_ = ref_pose_checkbox_->isChecked();
  p_record_velocity_ = velocity_checkbox_->isChecked();
  p_record_reference_velocity_ = ref_velocity_checkbox_->isChecked();
  p_record_controls_ = controls_checkbox_->isChecked();
}

void DataRecorderPanel::setParams() {
  nh_local_.setParam("active", p_active_);
  nh_local_.setParam("recording", p_recording_);

  nh_local_.setParam("record_pose", p_record_pose_);
  nh_local_.setParam("record_reference_pose", p_record_reference_pose_);
  nh_local_.setParam("record_velocity", p_record_velocity_);
  nh_local_.setParam("record_reference_velocity", p_record_reference_velocity_);
  nh_local_.setParam("record_controls", p_record_controls_);
}

void DataRecorderPanel::getParams() {
  nh_local_.param<bool>("active", p_active_, false);
  nh_local_.param<bool>("recording", p_recording_, false);

  nh_local_.param<bool>("record_pose", p_record_pose_, true);
  nh_local_.param<bool>("record_reference_pose", p_record_reference_pose_, true);
  nh_local_.param<bool>("record_velocity", p_record_velocity_, true);
  nh_local_.param<bool>("record_reference_velocity", p_record_reference_velocity_, true);
  nh_local_.param<bool>("record_controls", p_record_controls_, true);
}

void DataRecorderPanel::evaluateParams() {
  activate_checkbox_->setChecked(p_active_);

  if (!p_active_)
    p_recording_ = false;

  pose_checkbox_->setEnabled(p_active_ && !p_recording_);
  pose_checkbox_->setChecked(p_record_pose_);

  ref_pose_checkbox_->setEnabled(p_active_ && !p_recording_);
  ref_pose_checkbox_->setChecked(p_record_reference_pose_);

  velocity_checkbox_->setEnabled(p_active_ && !p_recording_);
  velocity_checkbox_->setChecked(p_record_velocity_);

  ref_velocity_checkbox_->setEnabled(p_active_ && !p_recording_);
  ref_velocity_checkbox_->setChecked(p_record_reference_velocity_);

  controls_checkbox_->setEnabled(p_active_ && !p_recording_);
  controls_checkbox_->setChecked(p_record_controls_);

  start_button_->setEnabled(p_active_ && !p_recording_);
  start_button_->setChecked(p_recording_);

  stop_button_->setEnabled(p_active_ && p_recording_);
  stop_button_->setChecked(!p_recording_);
}

void DataRecorderPanel::notifyParamsUpdate() {
  std_srvs::Empty empty;
  if (!params_cli_.call(empty)) {
    p_active_ = false;
    setParams();
    evaluateParams();
  }
}

void DataRecorderPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void DataRecorderPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robor_utilities::DataRecorderPanel, rviz::Panel)
