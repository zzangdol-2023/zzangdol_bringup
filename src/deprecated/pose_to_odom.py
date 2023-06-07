#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header

def pose_stamped_to_odometry(pose_stamped_msg):
    odometry_msg = Odometry()

    # Header 정보 설정
    odometry_msg.header = Header(stamp=pose_stamped_msg.header.stamp, frame_id='odom')
    odometry_msg.child_frame_id = 'base_footprint'

    # 포즈 정보 설정
    odometry_msg.pose.pose.position = pose_stamped_msg.pose.position
    odometry_msg.pose.pose.orientation = pose_stamped_msg.pose.orientation

    # Twist 정보는 0으로 설정
    odometry_msg.twist.twist.linear.x = 0.0
    odometry_msg.twist.twist.linear.y = 0.0
    odometry_msg.twist.twist.linear.z = 0.0
    odometry_msg.twist.twist.angular.x = 0.0
    odometry_msg.twist.twist.angular.y = 0.0
    odometry_msg.twist.twist.angular.z = 0.0

    return odometry_msg

# 메시지 콜백 함수
def pose_stamped_callback(pose_stamped_msg):
    # PoseStamped 메시지를 Odometry 메시지로 변환
    odometry_msg = pose_stamped_to_odometry(pose_stamped_msg)

    # 변환된 Odometry 메시지를 발행
    odometry_pub.publish(odometry_msg)

if __name__ == '__main__':
    # 노드 초기화
    rospy.init_node('pose2odom')

    # Subscriber 생성 및 콜백 함수 등록
    rospy.Subscriber('pose_stamped', PoseStamped, pose_stamped_callback)
    
    # Publisher 생성
    odometry_pub = rospy.Publisher('odom', Odometry, queue_size=10)

    # 노드 실행
    rospy.spin()
                         
