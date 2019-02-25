#!/usr/bin/env python
import rospy
import math
import tf
from tf.msg import tfMessage
import geometry_msgs.msg
from geometry_msgs.msg import Pose


def callback(data):
    #print("Received a TF message {}".format(data))
    
    # cast into tf message
    tf_msg = (tfMessage)(data)
    uav_pose = tf_msg.transforms
    print(tf_msg)
    #(uav_trans, uav_rot) =tf_msg.transforms
    #print(uav_trans[0])
    #print(uav_rot[3])

def tf_listen():

    # create topic
    pose_publisher = rospy.Publisher('uav_pose', Pose, queue_size = 10)

    # init node
    rospy.init_node('uav_pose_publisher', anonymous = True)

    # listen to /tf and get uav pose
    tf_subscriber = tf.TransformListener()

    rate = rospy.Rate(100.0)

    while not rospy.is_shutdown():
        try:
            # get uav pose
            (uav_trans, uav_rot) = tf_subscriber.lookupTransform('/world', 'uav/imu', rospy.Time(0))
            print("rospy.Time(0) is: {}".format(rospy.Time))
 
            # create and publish message with uav Pose
            pose_msg = Pose()

            pose_msg.position.x = uav_trans[0]
            pose_msg.position.y = uav_trans[1]
            pose_msg.position.z = uav_trans[2]

            pose_msg.orientation.x = uav_rot[0]
            pose_msg.orientation.y = uav_rot[1]
            pose_msg.orientation.z = uav_rot[2]
            pose_msg.orientation.w = uav_rot[3]
            pose_publisher.publish(pose_msg)

            # WTF it needs this to work!
            rospy.loginfo(pose_msg)
            #print(uav_trans[0])
            #print(uav_rot[3])

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo('tf connection error')
            continue

        rate.sleep()



if __name__ == '__main__':
    try:
        tf_listen()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass
