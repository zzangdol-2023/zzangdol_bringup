/*
 * cmd_vel_converter.cpp
 * Author: GeonhaPark<geonhab504@gmail.com>
 * Date: 2020.06.30
 * Description: cmd_vel_converter node which converts cmd_vel and cmd_vel_ehco topic to firmware accessible data
 */

#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define track_ 5.0
#define wheelbase_ 0.085
#define MAX_LINEAR_VEL 30
#define MIN_LINEAR_VEL 6
#define MAX_ANGLE 20
#define MIN_ANGLE 0
#define STEERING_DEGREE_WEIGHT 80.0
// prev value : 06.25 <
#define P_VEL 10
#define P_ANG -25
#define P_VEL_OFFSET 6.0 // Hardware goes up 6.0 signal

// last value : > 06.25
// #define P_VEL 22
// #define P_ANG -10

// Create a publisher for the cmd_vel topic
ros::Publisher cmd_vel_converter;
ros::Publisher cmd_vel_echo_converter;
ros::Subscriber cmd_vel_sub;
ros::Subscriber cmd_vel_echo_sub;
geometry_msgs::Twist converted;
geometry_msgs::Twist prev_converted;
geometry_msgs::Twist echo_converted;
geometry_msgs::Twist echo_prev_converted;
int back_driving_flag_cnt = 0;

/* =====================
* Convert Functions
===================== */
bool cmdVelEcho2Converted(const geometry_msgs::Twist::ConstPtr &cmd_vel_echo)
{
    // TODO : add ERROR handling logic => return false
    double raw_dc_pwm = cmd_vel_echo->linear.x;
    double raw_steering_angle = cmd_vel_echo->angular.z;
    double steering_radian = 0.0;
    double echo_linear_vel = 0.0;
    double echo_angular_vel = 0.0;
    echo_converted.linear.x = cmd_vel_echo->linear.x;
    echo_converted.angular.z = cmd_vel_echo->angular.z;

    /* get dc motor pwm value from cmd_vel_echo(pwm to ms)
     * PWM info
     * MAX : 20.0 -> 2.67 m/s
     * MIN : 5.0 -> 0 m/s
     *
     * PWM Topic | m/s
     * 20.0 | 2.67
     * 15.0 | 1.78
     * 10.0 | 0.89
     * 8.0 | 0.4 언저리
     * 5.0 | 0.0
     * result : y = 0.178(x-5.0) => linear equation.
     * x is pwm, y is m/s(linear.x)
     * x is integer value(5.0 6.0 7.0 ... 20.0 or -5.0 ... -20.0), y is float value.
     */

    echo_linear_vel = std::fabs(raw_dc_pwm);
    if ((int)echo_linear_vel < 5)
        echo_linear_vel = 5.0;
    echo_linear_vel = 0.178 * (echo_linear_vel - 5.0);
    // echo_linear_vel = round(100*echo_linear_vel)/100;
    if (std::signbit(raw_dc_pwm))
    {
        // ROS_INFO("cmd_vel_echo_converter <0 - echo_linear_vel: %f", -echo_linear_vel);
        echo_converted.linear.x = -echo_linear_vel;
    }
    else
    {
        // ROS_INFO("cmd_vel_echo_converter >0 - echo_linear_vel: %f", echo_linear_vel);
        echo_converted.linear.x = echo_linear_vel;
    }

    /* get steering angle data from cmd_vel_echo (angle to radian/s) */
    // ROS_INFO("cmd_vel_echo_converter - raw_steering_angle: %f", raw_steering_angle);
    // from angle to radian
    steering_radian = std::fabs(raw_steering_angle * (M_PI / 180.0));
    // ROS_INFO("cnd_vel_echo_converter - steering_radian: %f", steering_radian);
    // from radian to radian/s : v =rw -> w = v/r
    echo_angular_vel = echo_linear_vel / steering_radian;
    if (std::isnan(echo_angular_vel) || std::isinf(echo_angular_vel))
    {
        // ROS_INFO("cmd_vel_echo_converter - echo_angular_vel is nan");
        echo_angular_vel = 0.0;
    }
    if (std::signbit(raw_steering_angle))
    {
        // ROS_INFO("cmd_vel_echo_converter <0 - echo_angular_vel: %f", -echo_angular_vel);
        echo_converted.angular.z = -echo_angular_vel;
    }
    else
    {
        // ROS_INFO("cmd_vel_echo_converter >0 - echo_angular_vel: %f", echo_angular_vel);
        echo_converted.angular.z = echo_angular_vel;
    }

    return true;
}

bool cmdVel2Converted(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    double raw_linear_vel = cmd_vel->linear.x;
    double raw_angular_vel = cmd_vel->angular.z;

    // TODO : add ERROR handling logic => return false
    /* get steering angle data from cmd_vel (radian to angle) */
    double r = raw_linear_vel / raw_angular_vel;
    // ROS_INFO("cmd_vel_converter - radius : %f", r);
    // if (fabs(r) < track_ / 2.0)
    // {
    //     if (r == 0)
    //         r = raw_angular_vel / fabs(raw_angular_vel) * (track_ / 2.0 + 0.01);
    //     else
    //         r = r / fabs(r) * (track_ / 2.0 + 0.01);
    // }
    double steering_radian = std::atan(wheelbase_ / r);
    double steering_degree = std::fabs(steering_radian * (180.0 / M_PI));
    steering_degree*=STEERING_DEGREE_WEIGHT;
    ROS_INFO("converted - Received steering_degree: %f\n", steering_degree);
    
    if ((int)steering_degree > MAX_ANGLE)
        steering_degree = MAX_ANGLE;
    if ((int)steering_degree <= MIN_ANGLE)
        steering_degree = MIN_ANGLE;

    if (std::signbit(raw_angular_vel))
    {
        // ROS_INFO("cmd_vel_converter <0 - steering_degree: %f", -steering_degree);
        converted.angular.z = -steering_degree;
    }
    else
    {
        // ROS_INFO("cmd_vel_converter >0 - steering_degree: %f", steering_degree);
        converted.angular.z = steering_degree;
    }

    /* get dc motor pwm value (ms to pwm)
     * PWM info
     * MAX : 20.0 -> 2.67 m/s
     * MIN : 5.0 -> 0 m/s
     *
     * PWM Topic | m/s
     * 20.0 | 2.67
     * 15.0 | 1.78
     * 10.0 | 0.89
     * 8.0 | 0.4 언저리
     * 5.0 | 0.0
     * result : y = 0.178(x-5.0) => linear equation.
     * x = y/0.178+5.0 => y가 m/s(linear.x) x가 topic.
     */
    double linear_vel = std::fabs(raw_linear_vel);
    linear_vel = (linear_vel / 0.178) + 5.0;
    if ((int)linear_vel > MAX_LINEAR_VEL)
        linear_vel = MAX_LINEAR_VEL;
    if ((int)linear_vel <= MIN_LINEAR_VEL)
        linear_vel = 0;

    if (std::signbit(raw_linear_vel))
    {
        // ROS_INFO("cmd_vel_converter <0 - linear_vel: %f", -linear_vel);
        // back_driving buffer logic
        if (back_driving_flag_cnt < 2)
        {
            back_driving_flag_cnt++;
            converted.linear.x = -8.0;
        }
        else if (back_driving_flag_cnt < 5)
        {
            back_driving_flag_cnt++;
            converted.linear.x = 0.0;
        }
        else
        {
            converted.linear.x = -linear_vel;
        }
    }
    else
    {
        back_driving_flag_cnt = 0;
        // ROS_INFO("cmd_vel_converter >0 - linear_vel: %f", linear_vel);
        converted.linear.x = linear_vel;
    }

    return true;
}

/* =====================
* CallBack Functions
===================== */
void cmdVelEchoCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel_echo)
{
    // Print the received linear and angular velocities
    // ROS_INFO("cmd_vel_echo_converter - Received linear velocity: %f", cmd_vel_echo->linear.x);
    // ROS_INFO("cmd_vel_echo_converter - Received angular velocity: %f", cmd_vel_echo->angular.z);

    // Convert data range
    if (cmdVelEcho2Converted(cmd_vel_echo))
    {
        // Print the converted linear and angular velocities
        // ROS_INFO("cmd_vel_echo_converter - converted linear velocity: %f", echo_converted.linear.x);
        // ROS_INFO("cmd_vel_echo_converter - converted angular velocity: %f", echo_converted.angular.z);
        cmd_vel_echo_converter.publish(echo_converted);
        echo_prev_converted = echo_converted;
    }
    else
    {
        // ROS_INFO("cmd_vel_echo_converter - convert FAILED, publish previous data");
        // ROS_INFO("cmd_vel_echo_converter - prev linear velocity: %f", echo_prev_converted.linear.x);
        // ROS_INFO("cmd_vel_echo_converter - prev angular velocity: %f", echo_prev_converted.angular.z);
        cmd_vel_echo_converter.publish(echo_prev_converted);
    }
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    // Print the received linear and angular velocities
    // ROS_INFO("cmd_vel_converter - Received linear velocity: %f", cmd_vel->linear.x);
    // ROS_INFO("cmd_vel_converter - Received angular velocity: %f", cmd_vel->angular.z);

    // Convert data range
    if (cmdVel2Converted(cmd_vel))
    {
        // Print the converted linear and angular velocities
        // ROS_INFO("cmd_vel_converter - converted linear velocity: %f", converted.linear.x);
        // ROS_INFO("cmd_vel_converter - converted angular velocity: %f", converted.angular.z);
        cmd_vel_converter.publish(converted);
        prev_converted = converted;
    }
    else
    {
        // ROS_INFO("cmd_vel_converter - convert FAILED, publish previous data");
        // ROS_INFO("cmd_vel_converter - prev linear velocity: %f", prev_converted.linear.x);
        // ROS_INFO("cmd_vel_converter - prev angular velocity: %f", prev_converted.angular.z);
        cmd_vel_converter.publish(prev_converted);
    }
}

void cmdVelEcho_simple_Callback(const geometry_msgs::Twist::ConstPtr &cmd_vel_echo)
{
    echo_converted.linear.x = (cmd_vel_echo->linear.x) / P_VEL;
    echo_converted.angular.z = (cmd_vel_echo->angular.z) / P_ANG;

    cmd_vel_echo_converter.publish(echo_converted);
}

void cmdVel_simple_Callback(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    cmd_vel->linear.x > 0
        ? converted.linear.x = (cmd_vel->linear.x) * P_VEL + P_VEL_OFFSET
        : converted.linear.x = (cmd_vel->linear.x) * P_VEL - P_VEL_OFFSET;
    converted.angular.z = (cmd_vel->angular.z) * P_ANG;

    cmd_vel_converter.publish(converted);
}

/* =====================
* main
===================== */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_converter");
    ros::NodeHandle nh;

    // Publish cmd_vel_conveted topic
    cmd_vel_converter = nh.advertise<geometry_msgs::Twist>("cmd_vel_converted", 10);
    // ROS_INFO("cmd_vel_converter - Setup publisher on cmd_vel_converted");

    // Publish cmd_vel_echo_converted topic
    cmd_vel_echo_converter = nh.advertise<geometry_msgs::Twist>("cmd_vel_echo_converted", 10);
    // ROS_INFO("cmd_vel_echo_converter - Setup publisher on cmd_vel_converted");

    // Subscribe to the cmd_vel topic
    cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmdVel_simple_Callback);
    // ROS_INFO("cmd_vel_converter - Setup subscriber on cmd_vel");

    // Subscribe to the cmd_vel topic
    cmd_vel_echo_sub = nh.subscribe("cmd_vel_echo", 10, cmdVelEcho_simple_Callback);
    // cmd_vel_echo_sub = nh.subscribe("cmd_vel_echo", 10, cmdVelEchoCallback);
    // ROS_INFO("cmd_vel_echo_converter - Setup subscriber on cmd_vel");

    // Enter the main event loop
    ros::spin();

    return 0;
}
