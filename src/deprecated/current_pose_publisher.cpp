/*
 * current_pose_publisher.cpp
 * Author: GeonhaPark<geonhab504@gmail.com>
 * Date: 2020.06.30
 * Description: current_pose_publisher.cpp
 */

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "current_pose_publisher");
    ros::NodeHandle nh;
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 1);
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener(tf_buffer);
    ROS_INFO("pose_publisher started");
    while (ros::ok())
    {
        try
        {
            geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = transform.transform.translation.x;
            pose_stamped.pose.position.y = transform.transform.translation.y;
            pose_stamped.pose.position.z = transform.transform.translation.z;
            pose_stamped.pose.orientation = transform.transform.rotation;

            pose_publisher.publish(pose_stamped);
        }
        catch (tf2::LookupException &e)
        {
            continue;
        }
        catch (tf2::ConnectivityException &e)
        {
            continue;
        }
        catch (tf2::ExtrapolationException &e)
        {
            continue;
        }

        // ros::spinOnce();
    }

    return 0;
}