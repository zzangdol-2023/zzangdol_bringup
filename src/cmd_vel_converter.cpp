#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define track_ 5.0
#define wheelbase_ 0.085
#define MAX_LINEAR_VEL 20
#define MIN_LINEAR_VEL 0

// Create a publisher for the cmd_vel topic
ros::Publisher cmd_vel_converter;
ros::Subscriber cmd_vel_sub;
geometry_msgs::Twist converted;
geometry_msgs::Twist prev_converted;

bool cmdVel2Converted(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    // TODO : add convert logic -> cmd_vel to firmware accessible data
    converted.linear.x = cmd_vel->linear.x;
    converted.angular.z = cmd_vel->angular.z;

    /* get steering angle data from cmd_vel (radian to angle) */
    double r = cmd_vel->linear.x / cmd_vel->angular.z;
    if (fabs(r) < track_ / 2.0)
    {
        if (r == 0)
            r = cmd_vel->angular.z / fabs(cmd_vel->angular.z) * (track_ / 2.0 + 0.01);
        else
            r = r / fabs(r) * (track_ / 2.0 + 0.01);
    }
    double steering_angle = std::atan(wheelbase_ / r);
    ROS_INFO("cmd_vel_converter - steering_angle: %f", steering_angle);

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
    double linear_vel = std::fabs(cmd_vel->linear.x);
    linear_vel = ((cmd_vel->linear.x) / 0.178) + 5.0;
    if ((int)linear_vel > MAX_LINEAR_VEL)
        linear_vel = MAX_LINEAR_VEL;
    if ((int)linear_vel < MIN_LINEAR_VEL)
        linear_vel = 0;

    if ((int)cmd_vel->linear.x < 0)
    {
        ROS_INFO("cmd_vel_converter - linear_vel: %f", -linear_vel);
        // converted.linear.x = -linear_vel;
    }
    else
    {
        ROS_INFO("cmd_vel_converter - linear_vel: %f", linear_vel);
        // converted.linear.x = linear_vel;
    }

    return true;
}
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    // Print the received linear and angular velocities
    ROS_INFO("cmd_vel_converter - Received linear velocity: %f", cmd_vel->linear.x);
    ROS_INFO("cmd_vel_converter - Received angular velocity: %f", cmd_vel->angular.z);

    // Convert data range
    if (cmdVel2Converted(cmd_vel))
    {
        // Print the converted linear and angular velocities
        ROS_INFO("cmd_vel_converter - converted linear velocity: %f", converted.linear.x);
        ROS_INFO("cmd_vel_converter - converted angular velocity: %f", converted.angular.z);
        cmd_vel_converter.publish(converted);
        prev_converted = converted;
    }
    else
    {
        ROS_INFO("cmd_vel_converter - convert FAILED, publish previous data");
        ROS_INFO("cmd_vel_converter - prev linear velocity: %f", prev_converted.linear.x);
        ROS_INFO("cmd_vel_converter - prev angular velocity: %f", prev_converted.angular.z);
        cmd_vel_converter.publish(prev_converted);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_converter");
    ros::NodeHandle nh;

    // Publish cmd_vel_conveted topic
    cmd_vel_converter = nh.advertise<geometry_msgs::Twist>("cmd_vel_converted", 10);
    ROS_INFO("cmd_vel_converter - Setup publisher on cmd_vel_converted");

    // Subscribe to the cmd_vel topic
    cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmdVelCallback);
    ROS_INFO("cmd_vel_converter - Setup subscriber on cmd_vel");

    // Enter the main event loop
    ros::spin();

    return 0;
}
