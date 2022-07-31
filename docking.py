#!/usr/bin/env python
import math

import avg
import rospy
import geometry_msgs.msg
import nav_msgs.msg
import apriltag_ros.msg
import numpy as np
import tf
from sensor_msgs.msg import Imu
from tf import transformations as t
import tools


class Docking():

    def __init__(self):
        rospy.init_node("Docking")
        #to control motors
        self.vel_pub = rospy.Publisher("cmd_vel",geometry_msgs.msg.Twist,queue_size=10)

        #rospy.sleep(6) #in source code, its purpose is said to be for waiting for the AprilTagDetectionNode

        #tag_sub is for getting average position angle
        self.tag_sub = rospy.Subscriber("tag_detections",apriltag_ros.msg.AprilTagDetectionArray,self.get_avg_position_angle_callback)  

        #imu_sub is to get IMU data to calculate actual_angle
        self.imu_sub = rospy.Subscriber("imu_topic",Imu,self.actual_angle)

        #odom_sub is to get /odom frame
        self.odom_sub = rospy.Subscriber("odom",nav_msgs.msg.Odometry,self.get_odom_pos)
        
        #TF listener to listen frames
        self.tf_listener = tf.TransformListener()

        #Links and Transforms #NEEDS TO BE CHANGED AT END
        self.base_link = "base_link"
        self.camera_link = "camera_link"
        self.optical_frame = "optical_frame"

        #camera_link->Base_link
        self.transform_cam_base = geometry_msgs.msg.Transform()

        #Optical->camera_link
        self.transform_optical_cam = geometry_msgs.msg.Transform()

        self.odom_transform = geometry_msgs.msg.Transform()

        #demo purposes
        self.tag_id = "tag_0"

        #init of links
        self.get_cameraLink_baseLink()
        self.get_optical_frame()

        self.start_avg = True #must be false in the end of the project

        self.find_tag = False
        
        #actual_angle
        self.angle = 0


        ##Need explanations
        self.try_more = True
        self.bumper_pressed = False

        #Average of several vars
        self.avg_pos = avg(10)
        self.avg_dock = avg(2)
        self.avg_x = avg(10)
        self.avg_y = avg(10)
        self.avg_yaw = avg(10)


        self.odom_pos = geometry_msgs.msg.Vector3()

##########################################################################################################################################################################
    """
    INITS OF LINKS
    """
    def get_cameraLink_baseLink(self):
        """""
        initialize of transform_cam_base 
        Camera_link->base_link 
        
        """""
        try:
            begin = rospy.Time.now()
            self.tf_listener.waitForTransform(self.camera_link,self.base_link,begin,rospy.Duration(5.0))
            self.tf_listener.lookupTransform(self.camera_link,self.base_link,begin,self.transform_cam_base)
        except Exception as ex:
            rospy.logerr(ex)
            rospy.sleep(1.0)
    
    def get_optical_frame(self):
        """""
        initialize of transform_optical_cam 
        optical_link->camera_link 

        """""
        try:
            begin = rospy.Time.now()
            self.tf_listener.waitForTransform(self.optical_frame,self.camera_link,begin,rospy.Duration(5.0))
            self.tf_listener.lookupTransform(self.optical_frame,self.camera_link,begin,self.transform_optical_cam)
        except Exception as ex:
            rospy.logerr(ex)  
            rospy.sleep(1.0)

##########################################################################################################################################################################
#INIT OF ANGLE AND ODOM_TRANSFORM WITH CALLBACKS

    """
    INIT OF ANGLE BY IMU DATA
    """
    def actual_angle(self,data):
        quat = data.orientation
        euler = self.get_euler(quat)
        yaw = euler[2]
        self.angle = yaw

    """
    INIT OF ODOM_TRANSFORM BY ODOM DATA
    """
    def get_odom_pos(self,data):

        self.odom_pos.x = data.pose.pose.position.x
        self.odom_pos.y = data.pose.pose.position.y

        quaternion = geometry_msgs.msg.Quaternion(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
        pose = geometry_msgs.msg.Vector3(data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z)

        self.odom_transform = geometry_msgs.msg.Transform(pose,quaternion)
##########################################################################################################################################################################

#DOCKING AND DIRECTION ANGLE CALCULATIONS

    """
    CALCULATING DOCKING_ANGLE
    """
    def docking_angle(self):

        transform = geometry_msgs.msg.Transform()
        try:
            self.tf_listener.lookupTransform(self.base_link,self.tag_id,rospy.Time(0),transform)
        except Exception as e:
            rospy.logerr(f"docking_angle() {e}")
            rospy.sleep(1)
        
        tags_cor = self.getOrigin(transform)
        
        map_tag_x = tags_cor[0]
        map_tag_y = tags_cor[1]
        map_tag_z = tags_cor[2]

        return (map_tag_y/map_tag_x)  
    
    """
    CALCULATING DIRECTION ANGLE
    """

    def get_direction_angle(self):
        transform = geometry_msgs.msg.TransformStamped()

        try:
            begin = rospy.Time.now()
            self.tf_listener.waitForTransform(self.tag_id,self.base_link,begin,rospy.Duration(5.0))
            self.tf_listener.lookupTransform(self.tag_id,self.base_link,rospy.Time(0),transform)
        except Exception as ex:
            rospy.logerr(ex)
            rospy.sleep(1)
        
        tags_cor = self.getOrigin(transform)
        tag_x = tags_cor[0]
        tag_y = tags_cor[1]

        wd_rad = math.atan2(tag_x/tag_y)

        return wd_rad
    
    """
    CALCULATING YAW ANGLE
    """

    def get_yaw_angle(self):
        yaw = geometry_msgs.msg.TransformStamped()

        try:
            begin = rospy.Time.now()
            self.tf_listener.waitForTransform(self.base_link,self.tag_id,rospy.Duration(3.0))
            self.tf_listener.lookupTransform(self.base_link,self.tag_id,yaw)
        except Exception as ex:
            rospy.logerr(ex)
            rospy.sleep(1.0) 
       
        #return  -(M_PI/2 + tf::getYaw(yaw.getRotation()));
        yaw_quaternion = yaw.transform.rotation

        #ai, aj, ak : Euler's roll, pitch and yaw angles
        euler_angles = tools.get_euler_angles(yaw_quaternion)
        
        #euler_angles = self.get_euler(yaw_quaternion)
        
        yaw = euler_angles[2]
        
        
        #ISSUE: M_PI ??
        return -(M_PI/2 + yaw)
    
    """
    RETURNS POINT OBJECT
    """
    def get_position(self):
        transform = geometry_msgs.msg.TransformStamped()

        try:
            begin = rospy.Time.now()
            zero = rospy.Time(0)
            self.tf_listener.waitForTransform(self.tag_id,self.base_link,begin,rospy.Duration(10.0))
            self.tf_listener.lookupTransform(self.tag_id,self.base_link,zero,transform)
        except Exception as ex:
            rospy.logerr(ex)
        

        pos = geometry_msgs.msg.Point()

        pos_arr = tools.getOrigin(transform)
        #pos_arr = self.getOrigin(transform)
        pos.x = pos_arr[0]
        
        pos.y = pos_arr[2]

        return pos
##########################################################################################################################################################################
    """
    CALLBACK FUNCTIONS
    """
    

    """
    GETS THE AVERAGE POSITION
    """
    def get_avg_position_angle_callback(self,data):

        #TEMP
        #if self.start_avg:
        if True:
            size= len(data.detections)

            if (size !=1):
                rospy.logerr(f"must have exactly one apriltag, currently {size}")
            else:

                detect = data.detections[0]
                pose = detect.pose
                b = pose.pose
                p = b.pose

                point = p.position
                x = point.x
                y = point.y
                z = point.z
                
                optical_tag_origin = geometry_msgs.msg.Vector3(x,y,z)

                optical_tag_quad = geometry_msgs.msg.Quaternion(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)

                optical_tag_trans = geometry_msgs.msg.Transform(optical_tag_origin,optical_tag_quad)

                tag_cam = tools.multiply_transform_objects()


                
##########################################################################################################################################################################
##########################################################################################################################################################################
##########################################################################################################################################################################


    """
    DOCKING STAGE
    """
    def docking(self):

        
        rospy.sleep(3)
        return
    
    

    def startFrontalDocking(self):
    
        self.start_avg = False
        self.avg_pos.flush_array()
        self.avg_dock.flush_array()
        self.start_avg = True
        rospy.sleep(1)
        self.docking()

def main():
    tp = Docking()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()

