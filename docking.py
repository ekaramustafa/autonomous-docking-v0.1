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

        self.avg_position_angle = 0.0
        self.avg_docking_angle = 0.0
        self.avg_position_x = 0
        self.avg_position_y = 0
        self.avg_yaw_angle = 0

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
        
        tags_vec = tools.get_translation_vector(tools.get_transformation_matrix(transform))
        
        map_tag_x = tags_vec.x
        map_tag_y = tags_vec.y
        map_tag_z = tags_vec.z

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
        
        tags_vec = tools.get_translation_vector(tools.get_transformation_matrix(transform))
        tag_x = tags_vec.x
        tag_y = tags_vec.y

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

        pos_vec = tools.get_translation_vector(tools.get_transformation_matrix(transform))
        pos.x = pos_vec.x
        
        pos.y = pos_vec.z

        return pos
##########################################################################################################################################################################
    """
    CALLBACK FUNCTIONS
    """

#INIT OF ANGLE AND ODOM_TRANSFORM WITH CALLBACKS

#FUNDAMENTAL FUNC
    def actual_angle(self,data):
        """
        INIT OF ANGLE BY IMU DATA
        """
        quat = data.orientation
        euler = self.get_euler(quat)
        yaw = euler[2]
        self.angle = yaw

#FUNDAMENTAL FUNC
    def get_odom_pos(self,data):
        """
        INIT OF ODOM_TRANSFORM BY ODOM DATA
        """        
        self.odom_pos.x = data.pose.pose.position.x
        self.odom_pos.y = data.pose.pose.position.y

        quaternion = geometry_msgs.msg.Quaternion(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
        pose = geometry_msgs.msg.Vector3(data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z)

        self.odom_transform = geometry_msgs.msg.Transform(pose,quaternion)

#FUNDAMENTAL FUNC
    def get_avg_position_angle_callback(self,data):
        """
        GETS THE AVERAGE POSITION
        """

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

                tag_cam = tools.multiply_transforms(tools.get_inversed_transform_object(optical_tag_trans),self.transform_cam_base)
                
                tag_base = tools.multiply_transforms(tag_cam,self.transform_cam_base)

                base_tag = tools.get_inverse_transform_object()

                
                cords_vec = tools.get_translation_vector(base_tag)
                xx = cords_vec.x
                yy = cords_vec.y
                zz = cords_vec.z
                yaw = tools.get_euler_angles(base_tag.rotation)[2]

                if(abs(xx)<0.00001):
                    rospy.logerr("Division through zero is not allowed! xx:{0},y::{1},z:{2}".format(xx,yy,zz))
                
                else:     
                    alpha_dock = math.atan2(yy/xx)
                    alpha_pos = alpha_dock - (M_PI/2 + yaw)
                    
                    if(math.isfinite(alpha_pos)):
                        self.avg_pos.new_value(alpha_pos)
                        self.avg_dock.new_value(alpha_dock)
                        self.avg_x.new_value(z)
                        self.avg_y.new_value(x)
                        self.avg_yaw.new_value(yaw+ (M_PI/2))

                        avg_pos_angle = self.avg_pos.avg()
                        avg_dock_angle = self.avg_dock.avg()
                        avg_yaw_angle = self.avg_yaw.avg()
                        avg_position_x = self.avg_x.avg()
                        avg_position_y = self.avg_y.avg()
                        
                        self.avg_position_angle = avg_pos_angle
                        self.avg_docking_angle = avg_dock_angle
                        self.avg_position_x = avg_position_x
                        self.avg_position_y = avg_position_y
                        self.avg_yaw_angle = avg_yaw_angle
            
##########################################################################################################################################################################
##########################################################################################################################################################################

#FUNDAMENTAL

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

