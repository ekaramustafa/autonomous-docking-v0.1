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
from vision.scripts.AprilTag.tools import Tools, paper_tools


class Docking():

    def __init__(self):
        rospy.init_node("Docking")
        #to control motors
        self.vel_pub = rospy.Publisher("cmd_vel",geometry_msgs.msg.Twist,queue_size=10)

        #rospy.sleep(6) #in source code, its purpose is said to be for waiting for the AprilTagDetectionNode

        #tag_sub is for getting average position angle
        self.tag_sub = rospy.Subscriber("tag_detections",apriltag_ros.msg.AprilTagDetectionArray,self.get_avg_position_angle_callback)

        #find_tag_sub is for finding the right tag // not priority
        self.find_tag_sub = rospy.Subscriber("tag_detections",apriltag_ros.msg.AprilTagDetectionArray,self.findTag)     

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

        #for tag_sub callback //not priority
        self.TAG_AVAILABLE = False

        ##Need explanations
        self.try_more = True
        self.bumper_pressed = False

        #Average of several vars
        self.avg_pos = avg(10)
        self.avg_dock = avg(2)
        self.avg_x = avg(10)
        self.avg_y = avg(10)
        self.avg_yaw = avg(10)

        #Note: in source code odom_pos is 2D Vector. In python modules not possible for now(?)
        self.odom_pos = geometry_msgs.msg.Vector3()





        ###OPTIMIZATION TRIALS
        self.tools = paper_tools() 

##########################################################################################################################################################################
##########################################################################################################################################################################
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

#####################################################################################
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
#####################################################################################


##########################################################################################################################################################################
##########################################################################################################################################################################
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
##########################################################################################################################################################################
##########################################################################################################################################################################

    """
    TOOLS = NEED TO BE IN THE DIFFERENT CLASS !!
    """
    def getOrigin(self,transform):
        #Matrix_for_t = 4*4 homogenous transformation matrix
        matrix_for_t = self.get_transformation_matrix(transform)
        
        #originated is translation vector of matrix_for_t 
        originated = t.translation_from_matrix(matrix_for_t)
        #originated = [x,y,z]
        x = originated[0]
        y = originated[1]
        z = originated[2]
        return [x,y,z]


    def getAbs(self,other):

        if other < 0:
            return (-1)*other
        else:
            return other
    
    def get_yaw_angle(self):
        yaw = geometry_msgs.msg.TransformStamped

        try:
            begin = rospy.Time.now()
            self.tf_listener.waitForTransform(self.base_link,self.tag_id,rospy.Duration(3.0))
            self.tf_listener.lookupTransform(self.base_link,self.tag_id,yaw)
        except Exception as ex:
            rospy.logerr(ex)
            rospy.sleep(1.0)
        #yaw has header, child_frame_id and transform
            #transfrom has translation & rotation
                #Rotation is Quaternion we will get yaw angle from Quaternion   
       
        #return  -(M_PI/2 + tf::getYaw(yaw.getRotation()));
        yaw_quaternion = yaw.transform.rotation

        #ai, aj, ak : Euler's roll, pitch and yaw angles
        euler_angles = self.get_euler(yaw_quaternion)
        yaw = euler_angles[2]
        
        
        #issue: M_PI ??
        return -(M_PI/2 + yaw)
    
    def get_euler(self,quat):
        #RETURNS EULER ANGLES
        return t.euler_from_quaternion(quat)


    def get_position(self):
        transform = geometry_msgs.msg.TransformStamped()

        try:
            begin = rospy.Time.now()
            zero = rospy.Time(0)
            self.tf_listener.waitForTransform(self.tag_id,self.base_link,begin,rospy.Duration(10.0))
            self.tf_listener.lookupTransform(self.tag_id,self.base_link,zero,transform)
        except Exception as ex:
            rospy.logerr(ex)
        
        pos = geometry_msgs.msg.Point

        pos_arr = self.getOrigin(transform)
        pos.x = pos_arr[0]
        
        pos.y = pos_arr[2]

        return pos
    
    def get_inv_matrix(self,Q):
            return np.linalg.inv(Q)

    def get_transformation_matrix(self,transform):
        quaternion = transform.rotation
        quaternion_arr = [quaternion.x,quaternion.y,quaternion.z,quaternion.w]
        
        rot_matrix = []
        rot_matrix = t.quaternion_matrix(quaternion_arr)

        translation = transform.translation


        first_row = [rot_matrix[0][0],rot_matrix[0][1],rot_matrix[0][2],translation.x]
        second_row = [rot_matrix[1][0],rot_matrix[1][1],rot_matrix[1][2],translation.y]
        third_row = [rot_matrix[2][0],rot_matrix[2][1],rot_matrix[2][2],translation.z]
        fourth_row = [0,0,0,1]
        
        transformation_matrix = np.array([first_row,second_row,third_row,fourth_row])


        return transformation_matrix


    def get_inv_transform_matrix(self,transform):
        transformation_matrix = self.get_transformation_matrix(transform)
        result_matrix = self.get_inv_matrix(transformation_matrix)

        return result_matrix

    def convert_matrix_to_transform(self,matrix):
        translation = self.get_translation_vector(matrix)
        rotation = self.get_rotation_matrix(matrix)
        newTransform = geometry_msgs.msg.Transform(translation,rotation)
        return newTransform


    def get_rotation_matrix(self,matrix):

        rotation_matrix = [
        [matrix[0][0],matrix[0][1],matrix[0][2]],
        [matrix[1][0],matrix[1][1],matrix[1][2]],
        [matrix[2][0],matrix[2][1],matrix[2][2]]]
        return rotation_matrix

    def get_translation_vector(self,matrix):

        x = matrix[0][3]
        y= matrix[1][3]
        z = matrix[2][3]
        translation = [x,y,z]
        return translation

    def get_inv_transform(self,transform):
        # TRASNFORM : GEOMETRYMSGS TRANSFORM
        temp = self.get_transformation_matrix(transform)
        # TEMP : LİST 4*4
        inv_transform_matrix = self.get_inv_matrix(temp)

        cords = self.get_translation_vector(inv_transform_matrix)
        x = cords[0]
        y = cords[1]
        z = cords[2]

        translation = geometry_msgs.msg.Vector3(x,y,z)
        #DEBUG
        rotation_matrix = self.get_rotation_matrix(inv_transform_matrix)
        
        result_quaternion = t.quaternion_from_matrix(rotation_matrix)

        result_transform = geometry_msgs.msg.Transform(translation,result_quaternion)
        
        return result_transform

    """
    CALLBACK FUNCTIONS
    """
    
    def get_avg_position_angle_callback(self,data):
        #for only one ID

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

                #p has 
                point = p.position
                x = point.x
                y = point.y
                z = point.z
                
                optical_tag_origin = geometry_msgs.msg.Vector3(x,y,z)

                optical_tag_quad = geometry_msgs.msg.Quaternion(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)

                optical_tag_trans = geometry_msgs.msg.Transform(optical_tag_origin,optical_tag_quad)
                
                #For matrix multiplication
                optical_tag_trans_inv_mtx = self.get_inv_transform_matrix(optical_tag_trans)
                ####

                transform_optical_cam_mtx = self.get_transformation_matrix(self.transform_optical_cam)

                tag_cam_matrix = np.matmul(optical_tag_trans_inv_mtx,transform_optical_cam_mtx)

                tag_cam = self.convert_matrix_to_transform(tag_cam_matrix)

                #For matrix multiplication
                transform_cam_base_mtx = self.get_transformation_matrix(self.transform_cam_base)
                tag_base_matrix = np.matmul(tag_cam_matrix,transform_cam_base_mtx)

                tag_base = self.convert_matrix_to_transform(tag_base_matrix)     
                
                base_tag = self.get_inv_transform(tag_base)

                base_tag_cor = self.getOrigin(base_tag)

                xx = base_tag_cor[0]
                yy = base_tag_cor[1]
                zz = base_tag_cor[2]
                euler_base_tag = self.get_euler(geometry_msgs.msg.Quaternion (base_tag.rotation))
                yaw = euler_base_tag[2]


    def findTag(self,data):
    
        return


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

