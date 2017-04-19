#ifndef ros_pilot_joy_JOY_H_
#define ros_pilot_joy_JOY_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ros_pilot/JoyCommand.h>
#include <ros_plane/Controller_Commands.h>
#include <fcu_common/State.h>
#include "gazebo_msgs/ModelState.h"

typedef enum {
  NORMAL,
  LOOP_UP,
  LOOP_DOWN,
  ROLL_LEFT,
  ROLL_RIGHT,
} command_state_t;

struct Axes
{
  int x;
  int y;
  int F;
  int z;
  int x_direction;
  int y_direction;
  int F_direction;
  int z_direction;
};

struct Max
{
  double roll;
  double pitch;

  double roll_rate;
  double pitch_rate;
  double yaw_rate;

  double aileron;
  double elevator;
  double rudder;

  double hdot;
  double Vadot;
};

struct Button
{
  int index;
  bool prev_value;
};

struct Buttons
{
  Button fly;
  Button mode;
  Button reset;
  Button pause;
  Button loop_up;
  Button roll_right;
  Button roll_left;
};

class Joy
{
  typedef sensor_msgs::Joy::_buttons_type ButtonType;

private:
  ros::NodeHandle nh_;
  ros::Publisher command_pub_;
  ros::Subscriber autopilot_command_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber mav_state_sub_;

  std::string namespace_;
  std::string command_topic_;
  std::string autopilot_command_topic_;
  std::string mav_state_topic_;

  std::string mav_name_;
  std::string gazebo_ns_;

  Axes axes_;

  bool override_autopilot_ = true;
  bool paused = true;
  // double equilibrium_thrust_;

  ros_pilot::JoyCommand command_msg_;
  ros_plane::Controller_Commands autopilot_command_;
  //  fcu_common::ExtendedCommand extended_command_msg_;
  sensor_msgs::Joy current_joy_;

  Max max_;
  Buttons buttons_;
  geometry_msgs::Pose reset_pose_;
  geometry_msgs::Twist reset_twist_;

  double current_h_c_;
  double current_Va_c_;
  double current_chi_c_;
  double last_time_;

  fcu_common::State mav_state_;

  command_state_t command_state_ = NORMAL;


  // double current_yaw_vel_;
  // double v_yaw_step_;
  // double mass_;

  void StopMav();
  void ResetMav();
  void PauseSimulation();
  void ResumeSimulation();

  void JoyCallback(const sensor_msgs::JoyConstPtr &msg);
  void APCommandCallback(const ros_plane::Controller_CommandsConstPtr& msg);
  void StateCallback(const fcu_common::StateConstPtr& msg);
  void Publish();

public:
  Joy();
};

#endif  // ros_pilot_joy_JOY_H_
