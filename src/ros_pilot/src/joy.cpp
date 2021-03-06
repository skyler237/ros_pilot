/*
 * Copyright 2017 James Jackson
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros_pilot/joy.h"
#include "gazebo_msgs/SetModelState.h"
#include "std_srvs/Empty.h"

Joy::Joy()
{
  ros::NodeHandle pnh("~");
  ros::NodeHandle namespace_nh(ros::this_node::getNamespace());

  pnh.param<std::string>("command_topic", command_topic_, "command");
  pnh.param<std::string>("autopilot_command_topic", autopilot_command_topic_, "autopilot_command");
  pnh.param<std::string>("mav_state_topic", mav_state_topic_, "mav_state");

  // Get global parameters
  double max_thrust;
  // namespace_nh.param<double>("mass", mass_, 3.61);
  // namespace_nh.param<double>("max_F", max_thrust, 64.50);
  // equilibrium_thrust_ = (mass_*9.80665) / max_thrust;
  namespace_nh.param<std::string>("mav_name", mav_name_,"tracking_mav");

  // Get Parameters from joystick configuration yaml
  pnh.param<std::string>("gazebo_namespace", gazebo_ns_, "");
  pnh.param<int>("x_axis", axes_.x, 0);
  pnh.param<int>("y_axis", axes_.y, 3);
  pnh.param<int>("F_axis", axes_.F, 1);
  pnh.param<int>("z_axis", axes_.z, 2);

  pnh.param<int>("x_sign", axes_.x_direction, 1);
  pnh.param<int>("y_sign", axes_.y_direction, 1);
  pnh.param<int>("F_sign", axes_.F_direction, -1);
  pnh.param<int>("z_sign", axes_.z_direction, 1);

  pnh.param<double>("max_aileron", max_.aileron, 15.0 * M_PI / 180.0);
  pnh.param<double>("max_elevator", max_.elevator, 25.0 * M_PI / 180.0);
  pnh.param<double>("max_rudder", max_.rudder, 15.0 * M_PI / 180.0);

  pnh.param<double>("max_roll_rate", max_.roll_rate, 360.0 * M_PI / 180.0);
  pnh.param<double>("max_pitch_rate", max_.pitch_rate, 360.0 * M_PI / 180.0);
  pnh.param<double>("max_yaw_rate", max_.yaw_rate, 360.0 * M_PI / 180.0);

  pnh.param<double>("max_roll_angle", max_.roll, 45.0 * M_PI / 180.0);
  pnh.param<double>("max_pitch_angle", max_.pitch, 45.0 * M_PI / 180.0);

  pnh.param<double>("max_hdot", max_.hdot, 10);
  pnh.param<double>("max_Vadot", max_.Vadot, 5);

  // TODO: Initialize command
  command_msg_.mode = pnh.param<int>("control_mode", (int)ros_pilot::JoyCommand::MODE_AUTOPILOT);

  pnh.param<double>("reset_pos_x", reset_pose_.position.x, 0.0);
  pnh.param<double>("reset_pos_y", reset_pose_.position.y, 0.0);
  pnh.param<double>("reset_pos_z", reset_pose_.position.z, 0.0);
  pnh.param<double>("reset_orient_x", reset_pose_.orientation.x, 0.0);
  pnh.param<double>("reset_orient_x", reset_pose_.orientation.y, 0.0);
  pnh.param<double>("reset_orient_x", reset_pose_.orientation.z, 0.0);
  pnh.param<double>("reset_orient_x", reset_pose_.orientation.w, 0.0);
  pnh.param<double>("reset_linear_twist_x", reset_twist_.linear.x, 0.0);
  pnh.param<double>("reset_linear_twist_y", reset_twist_.linear.y, 0.0);
  pnh.param<double>("reset_linear_twist_z", reset_twist_.linear.z, 0.0);
  pnh.param<double>("reset_angular_twist_x", reset_twist_.angular.x, 0.0);
  pnh.param<double>("reset_angular_twist_x", reset_twist_.angular.y, 0.0);
  pnh.param<double>("reset_angular_twist_x", reset_twist_.angular.z, 0.0);

  // Sets which buttons are tied to which commands
  pnh.param<int>("button_takeoff", buttons_.fly.index, 0);
  pnh.param<int>("button_mode", buttons_.mode.index, 1);
  pnh.param<int>("button_reset", buttons_.reset.index, 9);
  pnh.param<int>("button_pause", buttons_.pause.index, 8);
  // trick buttons
  pnh.param<int>("button_loop_up", buttons_.loop_up.index, 4);
  pnh.param<int>("button_roll_right", buttons_.roll_right.index, 7);
  pnh.param<int>("button_roll_left", buttons_.roll_left.index, 6);

  command_pub_ = nh_.advertise<ros_pilot::JoyCommand>(command_topic_, 10);

  command_msg_.x = 0;
  command_msg_.y = 0;
  command_msg_.z = 0;
  command_msg_.F = 0;
  // command_msg_.ignore = 0xFF;

  // current_yaw_vel_ = 0;

  namespace_ = nh_.getNamespace();
  // TODO: Maybe get rid of this topic... probably don't need -- instead just add a mode to have the autopilot use it's own control
  autopilot_command_sub_ = nh_.subscribe(autopilot_command_topic_, 10, &Joy::APCommandCallback, this);
  joy_sub_ = nh_.subscribe("joy", 10, &Joy::JoyCallback, this);
  mav_state_sub_ = nh_.subscribe("mav_state", 10, &Joy::StateCallback, this);
  buttons_.mode.prev_value = 0;
  buttons_.reset.prev_value = 0;

  current_Va_c_ = current_h_c_ = current_chi_c_ = 0.0;
}

void Joy::StopMav()
{
  command_msg_.x = 0;
  command_msg_.y = 0;
  command_msg_.z = 0;
  command_msg_.F = 0;
}

/* Resets the mav back to origin */
void Joy::ResetMav()
{
  ROS_INFO("Mav position reset.");
  ros::NodeHandle n;

  gazebo_msgs::ModelState modelstate;
  modelstate.model_name = (std::string)mav_name_;
  modelstate.reference_frame = (std::string) "world";
  modelstate.pose = reset_pose_;
  modelstate.twist = reset_twist_;

  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
  setmodelstate.request.model_state = modelstate;
  client.call(setmodelstate);
}

// Pauses the gazebo physics and time
void Joy::PauseSimulation()
{
  ROS_INFO("Simulation paused.");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  std_srvs::Empty pauseSim;
  client.call(pauseSim);
}

// Resumes the gazebo physics and time
void Joy::ResumeSimulation()
{
  ROS_INFO("Simulation resumed.");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  std_srvs::Empty resumeSim;
  client.call(resumeSim);
  last_time_ = ros::Time::now().toSec();
}

void Joy::APCommandCallback(const ros_plane::Controller_CommandsConstPtr& msg)
{
  autopilot_command_ = *msg;
}

void Joy::StateCallback(const fcu_common::StateConstPtr& msg)
{
  mav_state_ = *msg;
}

void Joy::JoyCallback(const sensor_msgs::JoyConstPtr &msg)
{
  double dt = ros::Time::now().toSec() - last_time_;
  last_time_ = ros::Time::now().toSec();

  static double preloop_theta = mav_state_.theta;
  static double preroll_phi = mav_state_.phi;



  current_joy_ = *msg;

  // Perform button actions
  // Resets the mav to the origin
  if (msg->buttons[buttons_.reset.index] == 0 && buttons_.reset.prev_value == 1)  // button release
  {
    ResetMav();
    command_state_ = NORMAL;
  }
  buttons_.reset.prev_value = msg->buttons[buttons_.reset.index];



  // Pauses/Unpauses the simulation
  if (msg->buttons[buttons_.pause.index] == 0 && buttons_.pause.prev_value == 1)  // button release
  {
    if (!paused)
    {
      PauseSimulation();
      paused = true;
    }
    else
    {
      ResumeSimulation();
      paused = false;
    }
  }
  buttons_.pause.prev_value = msg->buttons[buttons_.pause.index];

  // Trick buttons
  if (msg->buttons[buttons_.loop_up.index] == 1 && buttons_.loop_up.prev_value == 0)  // button press
  {
    command_state_ = LOOP_UP;
    preloop_theta = mav_state_.theta;
  }
  buttons_.loop_up.prev_value = msg->buttons[buttons_.loop_up.index];

  if (msg->buttons[buttons_.roll_right.index] == 1 && buttons_.roll_right.prev_value == 0)  // button press
  {
    command_state_ = ROLL_RIGHT;
    preroll_phi = mav_state_.phi;
  }
  buttons_.roll_right.prev_value = msg->buttons[buttons_.roll_right.index];

  if (msg->buttons[buttons_.roll_left.index] == 1 && buttons_.roll_left.prev_value == 0)  // button press
  {
    command_state_ = ROLL_LEFT;
    preroll_phi = mav_state_.phi;
  }
  buttons_.roll_left.prev_value = msg->buttons[buttons_.roll_left.index];



  if (msg->buttons[buttons_.mode.index] == 0 && buttons_.mode.prev_value == 1)
  {
    command_state_ = NORMAL;
    command_msg_.mode = (command_msg_.mode + 1) % 3;
    if (command_msg_.mode == ros_pilot::JoyCommand::MODE_AUTOPILOT)
    {
      ROS_INFO("Autopilot Control Mode");
    }
    if (command_msg_.mode == ros_pilot::JoyCommand::MODE_DIRECT_CONTROL)
    {
      ROS_INFO("Direct Control Mode");
    }
    else if (command_msg_.mode == ros_pilot::JoyCommand::MODE_STABLE_CONTROL)
    {
      ROS_INFO("Stable Control Mode");
      // Grab the current commanded values from the autopilot to begin with
      current_h_c_ = autopilot_command_.h_c;
      current_Va_c_ = autopilot_command_.Va_c;
      current_chi_c_ = autopilot_command_.chi_c;
    }
    else if (command_msg_.mode == ros_pilot::JoyCommand::MODE_DIRECT_CONTROL)
    {
      override_autopilot_ = true;
      ROS_INFO("Direct Control Mode");
    }
    else
    {
      ROS_INFO("Invalid Mode. Value=%d", command_msg_.mode);
    }
  }
  buttons_.mode.prev_value = msg->buttons[buttons_.mode.index];

  // calculate the output command from the joysticks

  if (command_state_ == NORMAL) {
    command_msg_.F = msg->axes[axes_.F] * axes_.F_direction;
    command_msg_.x = msg->axes[axes_.x] * axes_.x_direction;
    command_msg_.y = msg->axes[axes_.y] * axes_.y_direction;
    command_msg_.z = msg->axes[axes_.z] * axes_.z_direction;

    switch (command_msg_.mode)
    {
    case ros_pilot::JoyCommand::MODE_DIRECT_CONTROL:
      command_msg_.x *= max_.aileron;
      command_msg_.y *= max_.elevator;
      command_msg_.z *= max_.rudder;
      break;

    case ros_pilot::JoyCommand::MODE_STABLE_CONTROL:
      // Integrate all axes
      // (Remember that roll affects y velocity and pitch affects -x velocity)
      current_h_c_ += dt * max_.hdot * command_msg_.y;
      command_msg_.y = current_h_c_;

      current_chi_c_ += dt * max_.yaw_rate * -1.0*command_msg_.z;
      current_chi_c_ = fmod(current_chi_c_ + 2.0*M_PI, 2.0*M_PI);
      // Cast the value into a range between -pi and pi
      command_msg_.z = (current_chi_c_ > M_PI ? current_chi_c_ - 2.0*M_PI : current_chi_c_);
      // command_msg_.z = current_chi_c_;

      current_Va_c_ += dt * max_.Vadot * -1.0*command_msg_.F;
      command_msg_.F = current_Va_c_;

      static double prev_h_c = 0.0;
      static double prev_chi_c = 0.0;
      static double prev_Va_c = 0.0;

      // Print the update when any of the values change
      if (current_h_c_ != prev_h_c || command_msg_.z != prev_chi_c || current_Va_c_ != prev_Va_c) {
        ROS_INFO("Updating commanded values: h_c=%f, chi_c=%f, Va_c=%f", current_h_c_, command_msg_.z, current_Va_c_);
      }

      prev_h_c = current_h_c_;
      prev_chi_c = command_msg_.z;
      prev_Va_c = current_Va_c_;

      break;

    }
  }
  else if (command_state_ == LOOP_UP) {
    command_msg_.F = axes_.F_direction;
    command_msg_.y = max_.elevator * axes_.y_direction;
    command_msg_.x = 0.0;
    command_msg_.z = 0.0;

    // Exit when the angle of the mav is back to normal
    if(mav_state_.theta > preloop_theta-0.15 && mav_state_.theta < preloop_theta-0.1 && fabs(mav_state_.phi) < M_PI/2.0) {
      command_state_ = NORMAL;
    }
  }
  else if (command_state_ == ROLL_LEFT) {
    command_msg_.F = axes_.F_direction;
    command_msg_.y = max_.elevator * axes_.y_direction;
    command_msg_.x = max_.aileron * axes_.x_direction;
    command_msg_.z = 0.0;

    // Exit when the angle of the mav is back to normal
    if(mav_state_.phi < preroll_phi+0.15 && mav_state_.phi > preroll_phi+0.1) {
      command_state_ = NORMAL;
    }
  }
  else if (command_state_ == ROLL_RIGHT) {
    command_msg_.F = axes_.F_direction;
    command_msg_.y = 0.0;
    command_msg_.x = -1.0 * max_.aileron * axes_.x_direction;
    command_msg_.z = 0.0;

    // Exit when the angle of the mav is back to normal
    if(mav_state_.phi > preroll_phi-0.15 && mav_state_.phi < preroll_phi-0.1) {
      command_state_ = NORMAL;
    }

  }

  Publish();
}

void Joy::Publish()
{
  if(command_state_ != NORMAL) {
    ros_pilot::JoyCommand temp_command = command_msg_;
    temp_command.mode = ros_pilot::JoyCommand::MODE_DIRECT_CONTROL;
    command_pub_.publish(temp_command);
  }
  else {
    command_pub_.publish(command_msg_);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fcu_common_joy");
  Joy joy;

  ros::spin();

  return 0;
}
