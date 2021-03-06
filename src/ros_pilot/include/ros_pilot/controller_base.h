/**
 * @file controller_base.h
 *
 * Base class definition for autopilot controller in chapter 6 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <ros/ros.h>
#include <fcu_common/Command.h>
#include <fcu_common/State.h>
#include <geometry_msgs/Vector3.h>
#include <ros_plane/Controller_Commands.h>
#include <ros_plane/Controller_Internals.h>
#include <ros_pilot/JoyCommand.h>

#include <dynamic_reconfigure/server.h>
#include <ros_pilot/ControllerConfig.h>

namespace ros_pilot {

enum class alt_zones {
    TakeOff,
    Climb,
    Descend,
    AltitudeHold
};

class controller_base
{
public:
    controller_base();
    float spin();

protected:

    struct input_s{
        float Ts;               /** time step */
        float h;                /** altitude */
        float va;               /** airspeed */
        float phi;              /** roll angle */
        float theta;            /** pitch angle */
        float chi;              /** course angle */
        float p;                /** body frame roll rate */
        float q;                /** body frame pitch rate */
        float r;                /** body frame yaw rate */
        float Va_c;             /** commanded airspeed (m/s) */
        float h_c;              /** commanded altitude (m) */
        float chi_c;            /** commanded course (rad) */
        float phi_c;            /** commanded roll angle (rad) */
        bool phi_valid;         /** determines if phi_c or chi_c is used */
    };

    struct output_s{
        float theta_c;
        float delta_e;
        float phi_c;
        float delta_a;
        float delta_r;
        float delta_t;
        alt_zones current_zone;
    };

    struct params_s {
        double alt_hz;           /**< altitude hold zone */
        double alt_toz;          /**< altitude takeoff zone */
        double tau;
        double c_kp;
        double c_kd;
        double c_ki;
        double r_kp;
        double r_kd;
        double r_ki;
        double p_kp;
        double p_kd;
        double p_ki;
        double p_ff;
        double a_p_kp;
        double a_p_kd;
        double a_p_ki;
        double a_t_kp;
        double a_t_kd;
        double a_t_ki;
        double a_kp;
        double a_kd;
        double a_ki;
        double b_kp;
        double b_kd;
        double b_ki;
        double trim_e;
        double trim_a;
        double trim_r;
        double trim_t;
        double max_e;
        double max_a;
        double max_r;
        double max_t;
        double pwm_rad_e;
        double pwm_rad_a;
        double pwm_rad_r;
    };

    virtual void control(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber _vehicle_state_sub;
    ros::Subscriber _vehicle_attitude_sub;
    ros::Subscriber _autopilot_commands_sub;
    ros::Subscriber _joy_commands_sub;
    ros::Publisher _actuators_pub;
    ros::Publisher _internals_pub;
    ros::Timer _act_pub_timer;

    struct params_s                    _params;            /**< params */
    ros_plane::Controller_Commands _autopilot_commands;
    ros_pilot::JoyCommand _joy_commands;
    fcu_common::State _vehicle_state;

    void vehicle_state_callback(const fcu_common::StateConstPtr& msg);
    void vehicle_attitude_callback(const fcu_common::StateConstPtr& msg);
    void autopilot_commands_callback(const ros_plane::Controller_CommandsConstPtr& msg);
    void joy_commands_callback(const ros_pilot::JoyCommandConstPtr& msg);
    bool _autopilot_command_recieved;
    bool _joy_command_recieved;

    dynamic_reconfigure::Server<ros_pilot::ControllerConfig> _server;
    dynamic_reconfigure::Server<ros_pilot::ControllerConfig>::CallbackType _func;

    void reconfigure_callback(ros_pilot::ControllerConfig &config, uint32_t level);

    /**
    * Convert from deflection angle to pwm
    */
    void convert_to_pwm(struct output_s &output);

    /**
    * Publish the outputs
    */
    void actuator_controls_publish(const ros::TimerEvent &);
};
} //end namespace

#endif // CONTROLLER_BASE_H
