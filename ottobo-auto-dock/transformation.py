#!/usr/bin/env python3
import rospy
import tools
import apriltag_ros.msg
import math
import tf
import geometry_msgs.msg
import numpy

from sensor_msgs.msg import Imu

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()

    main_frame = "base_link"
    rotating_frame = "camera_imu_optical_frame"
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
            tf_listener.waitForTransform(main_frame,rotating_frame,rospy.Time(0),rospy.Duration(5.0))
            pos,quad = tf_listener.lookupTransform(main_frame,rotating_frame,rospy.Time(0))

            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            rotation_matrix = tools.get_rotation_matrix_from_quaternion(quad_obj)
            inv_rotation_matrix = tools.get_inverse_matrix(rotation_matrix)
            #inv_rotation_matrix = numpy.matmul() 
            inv_quad = tools.get_quaternion_from_transformation_matrix(inv_rotation_matrix)

            translation = [pos[0],pos[1],pos[2]]
            rotation = [inv_quad.x,inv_quad.y,inv_quad.z,inv_quad.w]
            
            tf_broadcaster.sendTransform(translation,rotation,rospy.Time.now(),"imu_link",main_frame)
            rospy.loginfo("Sent")
            
            rate.sleep()
