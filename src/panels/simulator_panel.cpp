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

#include "robor_utilities/panels/simulator_panel.h"

using namespace robor_utilities;

SimulatorPanel::SimulatorPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), nh_local_("simulator") {
  params_cli_ = nh_local_.serviceClient<std_srvs::Empty>("params");
  getParams();

  activate_checkbox_         = new QCheckBox("On/Off");
  continuous_angle_checkbox_ = new QCheckBox("Continuous angle");
  set_button_                = new QPushButton("Set");
  t_f_input_                 = new QLineEdit(QString::number(p_time_constant_));
  t_o_input_                 = new QLineEdit(QString::number(p_time_delay_));
  init_x_input_              = new QLineEdit(QString::number(p_init_x_));
  init_y_input_              = new QLineEdit(QString::number(p_init_y_));
  init_theta_input_          = new QLineEdit(QString::number(p_init_theta_));

  QFrame* lines[4];
  for (auto& line : lines) {
    line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
  }

  QString theta = QChar(0x03B8);
  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QGridLayout* time_layout = new QGridLayout;
  time_layout->addItem(margin, 0, 0, 1, 1);
  time_layout->addWidget(new QLabel("T<sub>f</sub>:"), 0, 1);
  time_layout->addWidget(t_f_input_, 0, 2);
  time_layout->addWidget(new QLabel("s, "), 0, 3);
  time_layout->addWidget(new QLabel("T<sub>o</sub>:"), 0, 4);
  time_layout->addWidget(t_o_input_, 0, 5);
  time_layout->addWidget(new QLabel("s"), 0, 6);
  time_layout->addItem(margin, 0, 7, 1, 1);

  QGridLayout* inits_layout = new QGridLayout;
  inits_layout->addItem(margin, 0, 0, 1, 1);
  inits_layout->addWidget(new QLabel("x<sub>0</sub>:"), 0, 1);
  inits_layout->addWidget(init_x_input_, 0, 2);
  inits_layout->addWidget(new QLabel("m, "), 0, 3);
  inits_layout->addWidget(new QLabel("y<sub>0</sub>:"), 0, 4);
  inits_layout->addWidget(init_y_input_, 0, 5);
  inits_layout->addWidget(new QLabel("m, "), 0, 6);
  inits_layout->addWidget(new QLabel(theta + "<sub>0</sub>:"), 0, 7);
  inits_layout->addWidget(init_theta_input_, 0, 8);
  inits_layout->addWidget(new QLabel("rad"), 0, 9);
  inits_layout->addItem(margin, 0, 10, 1, 1);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addWidget(lines[0]);
  layout->addWidget(continuous_angle_checkbox_);
  layout->addWidget(lines[1]);
  layout->addLayout(time_layout);
  layout->addWidget(lines[2]);
  layout->addLayout(inits_layout);
  layout->addWidget(lines[3]);
  layout->addWidget(set_button_);
  layout->setAlignment(layout, Qt::AlignCenter);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(continuous_angle_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(set_button_, SIGNAL(clicked()), this, SLOT(processInputs()));

  evaluateParams();
}

void SimulatorPanel::processInputs() {
  verifyInputs();
  setParams();
  evaluateParams();
  notifyParamsUpdate();
}

void SimulatorPanel::verifyInputs() {
  p_active_ = activate_checkbox_->isChecked();
  p_continuous_angle_ = continuous_angle_checkbox_->isChecked();

  try { p_time_constant_ = boost::lexical_cast<double>(t_f_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_time_constant_ = 0.0; t_f_input_->setText("0.0"); }

  try { p_time_delay_ = boost::lexical_cast<double>(t_o_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_time_delay_ = 0.0; t_o_input_->setText("0.0"); }

  try { p_init_x_ = boost::lexical_cast<double>(init_x_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_init_x_ = 0.0; init_x_input_->setText("0.0"); }

  try { p_init_y_ = boost::lexical_cast<double>(init_y_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_init_y_ = 0.0; init_y_input_->setText("0.0"); }

  try { p_init_theta_ = boost::lexical_cast<double>(init_theta_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_init_theta_ = 0.0; init_theta_input_->setText("0.0"); }
}

void SimulatorPanel::setParams() {
  nh_local_.setParam("active", p_active_);
  nh_local_.setParam("continuous_angle", p_continuous_angle_);

  nh_local_.setParam("time_constant", p_time_constant_);
  nh_local_.setParam("time_delay", p_time_delay_);

  nh_local_.setParam("init_x", p_init_x_);
  nh_local_.setParam("init_y", p_init_y_);
  nh_local_.setParam("init_theta", p_init_theta_);
}

void SimulatorPanel::getParams() {
  p_active_ = nh_local_.param("active", false);
  p_continuous_angle_ = nh_local_.param("continuous_angle", false);

  p_time_constant_ = nh_local_.param("time_constant", 0.0);
  p_time_delay_ = nh_local_.param("time_delay", 0.0);

  p_init_x_ = nh_local_.param("init_x", 0.0);
  p_init_y_ = nh_local_.param("init_y", 0.0);
  p_init_theta_ = nh_local_.param("init_theta", 0.0);
}

void SimulatorPanel::evaluateParams() {
  activate_checkbox_->setChecked(p_active_);
  continuous_angle_checkbox_->setEnabled(p_active_);
  continuous_angle_checkbox_->setChecked(p_continuous_angle_);

  set_button_->setEnabled(p_active_);
  t_f_input_->setEnabled(p_active_);
  t_o_input_->setEnabled(p_active_);
  init_x_input_->setEnabled(p_active_);
  init_y_input_->setEnabled(p_active_);
  init_theta_input_->setEnabled(p_active_);
}

void SimulatorPanel::notifyParamsUpdate() {
  std_srvs::Empty empty;
  if (!params_cli_.call(empty)) {
    p_active_ = false;
    setParams();
    evaluateParams();
  }
}

void SimulatorPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void SimulatorPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robor_utilities::SimulatorPanel, rviz::Panel)
