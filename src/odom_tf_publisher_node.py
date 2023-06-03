#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3
import tf2_ros
import tf

tf_broadcaster = tf2_ros.TransformBroadcaster()

# odometry 데이터를 게시하기 위한 Publisher 생성
pub = rospy.Publisher('odom', Odometry, queue_size=10)
odom = Odometry()

def imu_callback(data):
    # Quaternion을 Euler로 변환하여 translation 값 추출
    euler = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    # odometry 데이터 생성
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_footprint'
    odom.pose.pose.position.x = data.linear_acceleration.x  # translation.x에 해당하는 값
    odom.pose.pose.position.y = data.linear_acceleration.y  # translation.y에 해당하는 값
    odom.pose.pose.position.z = 0  # translation.z에 해당하는 값
    odom.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0, yaw))

    
    odom_tf=tf2_ros.TransformStamped()
    odom_tf.header.stamp = odom.header.stamp
    odom_tf.header.frame_id = odom.header.frame_id
    odom_tf.child_frame_id = odom.child_frame_id
    odom_tf.transform.translation.x = odom.pose.pose.position.x
    odom_tf.transform.translation.y = odom.pose.pose.position.y
    odom_tf.transform.translation.z = odom.pose.pose.position.z
    odom_tf.transform.rotation = odom.pose.pose.orientation
    # odometry 데이터 발행
    pub.publish(odom)
    tf_broadcaster.sendTransform(
        odom_tf
    )

    # 로그 출력
    rospy.loginfo("Odometry published")


class MyROSNode:
    def __init__(self):
        # 노드 초기화
        rospy.init_node('imu_odom_publisher', anonymous=True)
        # IMU 데이터를 수신하기 위한 Subscriber 생성
        rospy.Subscriber('/imu/data', Imu, imu_callback)
    def run(self):
        rate = rospy.Rate(10)  # 1 Hz

        while not rospy.is_shutdown():
            # Do some processing or tasks
            rate.sleep()

if __name__ == '__main__':
    try:
        node = MyROSNode()
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
    #}
    #
    # Create a transform message