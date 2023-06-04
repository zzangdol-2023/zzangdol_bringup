#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3, Twist
import numpy as np
import tf2_ros
import tf
import time


class odom_tf_publisher_node:
    def __init__(self):
        # tf broadcaster, pulisher 선언
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.odom = Odometry()
        self.odom_tf = tf2_ros.TransformStamped()
        
        # odometry header 초기화
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footprint"
        self.odom_tf.header.frame_id = self.odom.header.frame_id
        self.odom_tf.child_frame_id = self.odom.child_frame_id
        
        # odometry 데이터 초기화
        self.cmd_vel = Twist()
        self.delta_s = 0
        self.theta = 0
        self.last_theta = 0
        self.delta_theta = 0
        self.cur_theta = 0
        self.odom_pose = [0.0, 0.0]
        
        # timer init
        self.timer = [0.0, 0.0]
        for i in range(2):
            self.timer[i] = time.time()
        
        
        # 노드 초기화
        rospy.init_node("odom_tf_publisher", anonymous=True)

        # IMU 데이터를 수신하기 위한 Subscriber 생성
        rospy.Subscriber("/imu/data", Imu, self.imuCallBackFunction)
        rospy.Subscriber("/cmd_vel", Twist, self.cmdVelCallbackFunction)


        # odometry 데이터를 게시하기 위한 Publisher 생성
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            ## Do some processing or tasks
            cur_time = time.time()
        
            # rospy.loginfo(f"timer:  {self.timer[0]}, cur_time: {cur_time}")
            
            # check data validity    
            if(cur_time-self.timer[0] > 0.1):
                rospy.loginfo("cmd_vel is not received")
                self.delta_s = 0
        
            # odometry 데이터 발행
            self.odom_pub.publish(self.odom)
            self.tf_broadcaster.sendTransform(self.odom_tf)
            rate.sleep()
            
    def cmdVelCallbackFunction(self, cmd_vel_data):
        self.timer[0] = time.time()
        self.cmd_vel = cmd_vel_data
        self.delta_s = self.cmd_vel.linear.x * 0.1  # 0.1 rospy.Rate(10)의 주기
        # 로그 출력
        # rospy.loginfo(self.cmd_vel)
        
        pass
    
    def imuCallBackFunction(self, imu_data):
        self.timer[1] = time.time()
        step_time = 0.1
        # Quaternion을 Euler로 변환하여 translation 값 추출
        euler = tf.transformations.euler_from_quaternion(
            [
                imu_data.orientation.x,
                imu_data.orientation.y,
                imu_data.orientation.z,
                imu_data.orientation.w,
            ]
        )
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        # rospy.loginfo(f"roll: {roll}, pitch: {pitch}, yaw: {yaw}")
        
        self.theta       = yaw
        self.delta_theta = self.theta - self.last_theta
        
        #  compute odometric pose
        self.odom_pose[0] = self.odom_pose[0] + self.delta_s * np.cos(self.cur_theta + (self.delta_theta / 2.0))
        self.odom_pose[1] = self.odom_pose[1] + self.delta_s * np.sin(self.cur_theta + (self.delta_theta / 2.0))
        self.cur_theta = self.cur_theta + self.delta_theta
        
        # compute odometric instantaneouse velocity
        v = self.delta_s / step_time
        w = self.delta_theta / step_time
        
        # odometry 데이터 생성
        self.odom.header.stamp = rospy.Time.now()
        
        # translation.x에 해당하는 값
        # self.odom.pose.pose.position.x = self.odom.pose.pose.position.x + (
        #     imu_data.linear_acceleration.x * 0.1 * 0.1
        # )  
        self.odom.pose.pose.position.x = self.odom_pose[0]
        
        # translation.y에 해당하는 값
        # self.odom.pose.pose.position.y = self.odom.pose.pose.position.y + (
        #     imu_data.linear_acceleration.y * 0.1 * 0.1
        # )  
        self.odom.pose.pose.position.y = self.odom_pose[1]
        
        # translation.z에 해당하는 값
        self.odom.pose.pose.position.z = 0  
        
        self.odom.pose.pose.orientation = Quaternion(
            *tf.transformations.quaternion_from_euler(0, 0, yaw)
        )
        self.odom.twist.twist = Twist(
            linear=Vector3(v, 0, 0), angular=Vector3(0, 0, w)
        )
        # odometry 데이터를 tf 메시지로 변환
        self.odom_tf.header.stamp = self.odom.header.stamp
        self.odom_tf.transform.translation.x = self.odom.pose.pose.position.x
        self.odom_tf.transform.translation.y = self.odom.pose.pose.position.y
        self.odom_tf.transform.translation.z = self.odom.pose.pose.position.z
        self.odom_tf.transform.rotation = self.odom.pose.pose.orientation

        # set last_theta=theta
        self.last_theta = self.theta

        # 로그 출력
        # rospy.loginfo("IMU Data Received")
        pass


if __name__ == "__main__":
    try:
        node = odom_tf_publisher_node()
        node.run()

    except rospy.ROSInterruptException:
        pass

    ###
    # msg.TransformStamped
    # {
    #   header{
    #     stamp (rospyTime)
    #     frame_id (string : the frame id of the parent frame)
    #   },
    #   transform
    #   {
    #         child_frame_id (string :the frame id of the child frame)
    #         translation(x,y,z)
    #         rotation(x,y,z,w)
    #   }
    # }
    #
    # Create a transform message
