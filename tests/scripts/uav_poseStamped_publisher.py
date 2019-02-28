#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import PoseStamped


def publish_auv_poseStamped():

    # create topic
    pose_publisher = rospy.Publisher('uav_PoseStamped', PoseStamped, queue_size = 10)

    # init node
    rospy.init_node('uav_PoseStamped_publisher', anonymous = True)
    rospy.loginfo("Created node for UAV Pose Stamped publication")

    # start a tf subscriber
    tf_subscriber = tf.TransformListener()

    # wait for transform tree to complete... 
    # the first time might be problematic since transforms might not be already
    # published
    tf_subscriber.waitForTransform("/world", "uav/imu", rospy.Time(), rospy.Duration(4.0))

    # publish at 100hz
    rate = rospy.Rate(100.0)
    
    while not rospy.is_shutdown():

        try:
            # get time of latest available transform
            latest_time = tf_subscriber.getLatestCommonTime('/world', 'uav/imu')

            # get transform -translation,rotation- at latest time
            (uav_t, uav_r) = tf_subscriber.lookupTransform('/world', 'uav/imu', latest_time)

            # create and populate PoseStamped message
            ps_msg = PoseStamped()

            # Fill Header first
            ps_msg.header.stamp = rospy.Time.now() #latest_time , for sync
            ps_msg.header.frame_id = 'uav/imu'

            # Fill Translation
            ps_msg.pose.position.x = uav_t[0]
            ps_msg.pose.position.y = uav_t[1]
            ps_msg.pose.position.z = uav_t[2]

            # Fill Orientation
            ps_msg.pose.orientation.x = uav_r[0]
            ps_msg.pose.orientation.y = uav_r[1]
            ps_msg.pose.orientation.z = uav_r[2]
            ps_msg.pose.orientation.w = uav_r[3]

            pose_publisher.publish(ps_msg)
            rospy.loginfo(ps_msg)

        except (tf.LookupException, tf.ConnectivityException): #tf.ExtrapolationException
            rospy.loginfo('tf connection error')
            continue

        rate.sleep()


if __name__ == '__main__':
    try:
        publish_auv_poseStamped()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass
