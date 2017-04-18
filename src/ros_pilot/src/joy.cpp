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

  // Get global parameters
  double max_thrust;
  // namespace_nh.param<double>("mass", mass_, 3.61);
  // namespace_nh.param<double>("max_F", max_thrust, 64.50);
  // equilibrium_thrust_ = (mass_*9.80665) / max_thrust;
  namespace_nh.param<std::string>("mav_name", mav_name_,"junker");

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

  pnh.param<double>("max_hdot", max_.hdot, 1.5);
  pnh.param<double>("max_Vadot", max_.Vadot, 1.5);

  // TODO: Initialize command
  command_msg_.mode = pnh.param<int>("control_mode", (int)ros_pilot::JoyCommand::MODE_DIRECT_CONTROL);

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
  // TODO remove override ??
  pnh.param<int>("button_override", buttons_.override.index, 8);

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

void Joy::APCommandCallback(const ros_pilot::JoyCommandConstPtr &msg)
{
  autopilot_command_ = *msg;
}

void Joy::JoyCallback(const sensor_msgs::JoyConstPtr &msg)
{
  double dt = ros::Time::now().toSec() - last_time_;
  last_time_ = ros::Time::now().toSec();

  current_joy_ = *msg;

  // Perform button actions
  // Resets the mav to the origin
  if (msg->buttons[buttons_.reset.index] == 0 && buttons_.reset.prev_value == 1)  // button release
  {
    ResetMav();
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

  if (msg->buttons[buttons_.mode.index] == 0 && buttons_.mode.prev_value == 1)
  {
    command_msg_.mode = (command_msg_.mode + 1) % 2;
    if (command_msg_.mode == ros_pilot::JoyCommand::MODE_DIRECT_CONTROL)
    {
      override_autopilot_ = true;
      ROS_INFO("Direct Control Mode");
    }
    else if (command_msg_.mode == ros_pilot::JoyCommand::MODE_STABLE_CONTROL)
    {
      override_autopilot_ = true;
      ROS_INFO("Stable Control Mode");
    }
    else
    {
      ROS_INFO("Invalide Mode");
    }
  }
  buttons_.mode.prev_value = msg->buttons[buttons_.mode.index];

  // calculate the output command from the joysticks
  if(override_autopilot_)
  {
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

      current_chi_c_ += dt * max_.yaw_rate * command_msg_.z;
      current_chi_c_ = fmod(current_chi_c_, (2.0 * M_PI));
      command_msg_.z = current_chi_c_;

      current_Va_c_ += dt * max_.Vadot * command_msg_.F;
      command_msg_.F = current_Va_c_;

      ROS_INFO("Updating commanded values: h_c=%f, chi_c=%f, Va_c=%f", current_h_c_, current_chi_c_, current_Va_c_);
      break;

    // case ros_pilot::JoyCommand::MODE_STABLE_CONTROL:
    //   command_msg_.x *= max_.roll;
    //   command_msg_.y *= max_.pitch;
    //   command_msg_.z *= max_.yaw_rate;
    //   if (command_msg_.F > 0.0)
    //   {
    //     command_msg_.F = equilibrium_thrust_ + (1.0 - equilibrium_thrust_) * command_msg_.F;
    //   }
    //   else
    //   {
    //     command_msg_.F = equilibrium_thrust_ + (equilibrium_thrust_) * command_msg_.F;
    //   }
    //   break;

    // case ros_pilot::JoyCommand::MODE_ROLL_PITCH_YAWRATE_ALTITUDE:
    //   command_msg_.x *= max_.roll;
    //   command_msg_.y *= max_.pitch;
    //   command_msg_.z *= max_.yaw_rate;
    //   // Integrate altitude
    //   current_altitude_setpoint_ -= dt * max_.Vadot * command_msg_.F;
    //   command_msg_.F = current_altitude_setpoint_;
    //   break;
    //
    // case ros_pilot::JoyCommand::MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
    // {
    //   // Remember that roll affects y velocity and pitch affects -x velocity
    //   double original_x = command_msg_.x;
    //   command_msg_.x = max_.hdot * -1.0 * command_msg_.y;
    //   command_msg_.y = max_.yvel * original_x;
    //   command_msg_.z *= max_.yaw_rate;
    //   // Integrate altitude
    //   current_altitude_setpoint_ -= dt * max_.Vadot * command_msg_.F;
    //   command_msg_.F = current_altitude_setpoint_;
    //   break;
    // }


    }
  }
  else
  {
    command_msg_ = autopilot_command_;
  }

  Publish();
}

void Joy::Publish()
{
  command_pub_.publish(command_msg_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fcu_common_joy");
  Joy joy;

  ros::spin();

  return 0;
}
