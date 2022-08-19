#!/usr/bin/env python3
import math

from avg import avg
import rospy
import geometry_msgs.msg
import nav_msgs.msg
import apriltag_ros.msg
import tf
from sensor_msgs.msg import Imu
import tools
import math
#TO-DO

class Docking():

    def __init__(self):
        rospy.init_node("Docking")
        #to control motors
        self.vel_pub = rospy.Publisher("cmd_vel",geometry_msgs.msg.Twist,queue_size=10)

        rospy.loginfo("DOCKING INIT")
        rospy.sleep(2) #in source code, its purpose is said to be for waiting for the AprilTagDetectionNode
        
        #TF listener to listen frames
        self.tf_listener = tf.TransformListener()
        
        self.base_link = "base_link"
        self.camera_link = "camera_link"
        self.optical_frame = "camera_color_optical_frame"

        #camera_link->Base_link
        self.transform_cam_base = geometry_msgs.msg.Transform()

        #Optical->camera_link
        self.transform_optical_cam = geometry_msgs.msg.Transform()

        self.odom_transform = geometry_msgs.msg.Transform()

        """
        MODIFY THE TAG_ID
        """
        self.tag_id = "tag_0"

        #init of links
        self.get_cameraLink_baseLink()
        rospy.loginfo("[INIT] CameraLink -> baseLink")
        self.get_optical_frame()
        rospy.loginfo("[INIT] optical -> camera")

        #is a bool to start and stop get_avg_position_callback
        self.start_avg = False 

        #is a bool to start and stop findTag callback
        self.find_tag = False
        
        #yaw angle from IMU sensor. The IMU is a built-in sensor that can
        #measure the angle the robot has turned. 
        self.angle = 0

        #used in findTag callback
        self.TAG_AVAILABLE = False

        #to keep trying after failed attempts
        self.try_more = True

        #Average of several vars
        #NEED MORE EXPLANATION
        #params
        rospy.loginfo("[INIT] avg inits")
        self.avg_pos = avg(10)
        self.avg_dock = avg(2)
        self.avg_x = avg(10)
        self.avg_y = avg(10)
        self.avg_z = avg(10)

        self.avg_position_angle = 0.0
        self.avg_docking_angle = 0.0

        self.initial_pos_x = 0
        self.initial_pos_y = 0
        self.initial_docking_angle = 0
        self.initial_inits = False

        self.avg_position_x = 0.0
        self.avg_position_y = 0.0
        self.avg_position_z = 0.0

        self.yaw = 0


        self.docking_status = True

        self.M_PI = math.pi #180 degrees, pi in radians i suppose

        self.odom_pos = geometry_msgs.msg.Vector3()


        #tag_sub is for getting average position angle
        self.tag_sub = rospy.Subscriber("tag_detections",apriltag_ros.msg.AprilTagDetectionArray,self.get_avg_position_angle_callback)  

        #imu_sub is to get IMU data to calculate actual_angle
        self.imu_sub = rospy.Subscriber("imu/data",Imu,self.actual_angle)

        #find_tag_sub is to search for specific tag_id 
        self.find_tag_sub = rospy.Subscriber("tag_detections",apriltag_ros.msg.AprilTagDetectionArray,self.findTag)

        #odom_sub is to get /odom frame
        self.odom_sub = rospy.Subscriber("odom",nav_msgs.msg.Odometry,self.get_odom_pos)


        
        rospy.loginfo("[INIT] Start Docking Process")
        self.startDocking()

##################################################################
################## INIT OF LINKS #################################
##################################################################

    """
    INITS OF LINKS
    """
    def get_cameraLink_baseLink(self):
        """""
        initialize transform_cam_base 
        Camera_link->base_link 
        
        """""
        try:
            begin = rospy.Time.now()
            self.tf_listener.waitForTransform(self.camera_link,self.base_link,rospy.Time(0),rospy.Duration(5.0))
            pos, quad= self.tf_listener.lookupTransform(self.camera_link,self.base_link,rospy.Time(0))
            pos_obj = geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2])
            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            self.transform_cam_base = geometry_msgs.msg.Transform(pos_obj,quad_obj)
        except Exception as ex:
            rospy.logerr(ex)
            rospy.sleep(1.0)
    
    def get_optical_frame(self):
        """""
        initialize transform_optical_cam 
        optical_link->camera_link 

        """""
        try:
            begin = rospy.Time.now()
            self.tf_listener.waitForTransform(self.optical_frame,self.camera_link,rospy.Time(0),rospy.Duration(5.0))
            pos, quad =self.tf_listener.lookupTransform(self.optical_frame,self.camera_link,rospy.Time(0))
            pos_obj = geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2])
            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            self.transform_optical_cam = geometry_msgs.msg.Transform(pos_obj,quad_obj)
        except Exception as ex:
            rospy.logerr(ex)  
            rospy.sleep(1.0)

##################################################################
################## CALCULATING ANGLES ############################
##################################################################
    """
    CALCULATING DOCKING_ANGLE
    """
    def docking_angle(self):

        transform = geometry_msgs.msg.Transform()
        
        try:
            begin = rospy.Time.now()
            self.tf_listener.waitForTransform(self.base_link,self.tag_id,begin,rospy.Duration(5.0))
            pos, quad = self.tf_listener.lookupTransform(self.base_link,self.tag_id,rospy.Time(0))
            pos_obj = geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2])
            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            transform = geometry_msgs.msg.Transform(pos_obj,quad_obj)
        
        except Exception as e:
            rospy.logerr(f"docking_angle() {e}")
            rospy.sleep(1)
        
        tags_vec = tools.get_translation_vector(tools.get_transformation_matrix(transform))
        
        map_tag_x = tags_vec.x
        map_tag_y = tags_vec.y
        map_tag_z = tags_vec.z

        if not self.initial_inits:
            self.initial_pos_y = map_tag_y
            self.initial_docking_angle = (map_tag_y/map_tag_x)
            self.initial_inits = True 
        
        return (map_tag_y/map_tag_x)  
    
    def get_point_base_tag(self):

        transform = geometry_msgs.msg.Transform()
        
        try:
            begin = rospy.Time.now()
            self.tf_listener.waitForTransform(self.base_link,self.tag_id,begin,rospy.Duration(5.0))
            pos, quad = self.tf_listener.lookupTransform(self.base_link,self.tag_id,rospy.Time(0))
            pos_obj = geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2])
            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            transform = geometry_msgs.msg.Transform(pos_obj,quad_obj)
        
        except Exception as e:
            rospy.logerr(f"docking_angle() {e}")
            rospy.sleep(1)
        
        tags_vec = tools.get_translation_vector(tools.get_transformation_matrix(transform))
        
        map_tag_x = tags_vec.x
        map_tag_y = tags_vec.y
        map_tag_z = tags_vec.z
        
        return geometry_msgs.msg.Point(map_tag_x,map_tag_y,map_tag_z)  

    
    """
    CALCULATING DIRECTION ANGLE
    """

    def get_direction_angle(self):
        transform = geometry_msgs.msg.TransformStamped()

        try:
            begin = rospy.Time.now()
            self.tf_listener.waitForTransform(self.tag_id,self.base_link,begin,rospy.Duration(5.0))
            pos, quad = self.tf_listener.lookupTransform(self.tag_id,self.base_link,rospy.Time(0))
            pos_obj = geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2])
            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            transform = geometry_msgs.msg.Transform(pos_obj,quad_obj)
        except Exception as ex:
            rospy.logerr(ex)
            rospy.sleep(1)
        
        tags_vec = tools.get_translation_vector(tools.get_transformation_matrix(transform))
        tag_x = tags_vec.x
        tag_y = tags_vec.z

        wd_rad = math.atan(tag_x/tag_y)

        return wd_rad
    
    """
    CALCULATING YAW ANGLE
    """

    def get_yaw_angle(self):
        yaw = geometry_msgs.msg.TransformStamped()

        try:
            begin = rospy.Time.now()
            self.tf_listener.waitForTransform(self.base_link,self.tag_id,begin,rospy.Duration(3.0))
            pos,quad = self.tf_listener.lookupTransform(self.base_link,self.tag_id,begin)
            pos_obj = geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2])
            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            yaw = geometry_msgs.msg.Transform(pos_obj,quad_obj)
        except Exception as ex:
            rospy.logerr(ex)
            rospy.sleep(1.0) 
       
        yaw_quaternion = yaw.rotation

        #ai, aj, ak : Euler's roll, pitch and yaw angles
        euler_angles = tools.get_euler_angles(yaw_quaternion)
                
        yaw = euler_angles[2]
        
        return -(self.M_PI/2 + yaw)
    
##################################################################
################## CALLBACK FUNCTIONS ############################
##################################################################

#FUNDAMENTAL FUNC
    def findTag(self,data):
        if self.find_tag:  
            for i in range(len(data.detections)):
                tag_id, = data.detections[i].id
                #should check hz 
                if self.tag_id == "tag_{}".format(tag_id):
                    X = rospy.Duration(2.0)
                    time = data.detections[i].pose.header.stamp
                    now = rospy.Time.now()
                    age = now-time

                    if X > age:
                        self.TAG_AVAILABLE = True
                        rospy.loginfo("Tag is avaliable")

#FUNDAMENTAL FUNC
    def actual_angle(self,data):
        """
        INIT OF ANGLE BY IMU DATA
        """
        quat = data.orientation
        euler = tools.get_euler_angles(quat)
        yaw = euler[2]
        if yaw < 0:
            self.angle = yaw + 2*self.M_PI
        else:
            self.angle = yaw

#FUNDAMENTAL FUNC
    def get_odom_pos(self,data):
        """
        INIT OF ODOM_TRANSFORM BY ODOM DATA
        """        
        self.odom_pos.x = data.pose.pose.position.x
        self.odom_pos.y = data.pose.pose.position.y

        quaternion = geometry_msgs.msg.Quaternion(data.pose.pose.orientation.x,
                                                    data.pose.pose.orientation.y,
                                                    data.pose.pose.orientation.z,
                                                    data.pose.pose.orientation.w)

        pose = geometry_msgs.msg.Vector3(data.pose.pose.position.x,
                                            data.pose.pose.position.y,
                                            data.pose.pose.position.z)

        self.odom_transform = geometry_msgs.msg.Transform(pose,quaternion)

#FUNDAMENTAL FUNC
    def get_avg_position_angle_callback(self,data):
        """
        GETS THE AVERAGE POSITION
        """
        if self.start_avg:
            for i in range(len(data.detections)):
                id, = data.detections[i].id
                if self.tag_id == "tag_{}".format(id): 
                    detect = data.detections[i]
                    pose = detect.pose
                    b = pose.pose
                    p = b.pose

                    point = p.position
                    x = point.x
                    y = point.y
                    z = point.z



                    #opticalFrame -> tagFrame
                    optical_tag_origin = geometry_msgs.msg.Vector3(x,y,z)

                    optical_tag_quad = geometry_msgs.msg.Quaternion(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)

                    # optical -> tag
                    optical_tag_trans = geometry_msgs.msg.Transform(optical_tag_origin,optical_tag_quad)
                    ##




                    #tag -> cam = (optical->tag)^(-1) * (optical->cam)  
                    tag_cam = tools.multiply_transforms(tools.get_inverse_transform_object(optical_tag_trans),self.transform_optical_cam)
                    
  

                    #tag -> base = (tag -> cam)* (cam->base)
                    tag_base = tools.multiply_transforms(tag_cam,self.transform_cam_base)



                    #base -> tag = (tag -> base)^(-1)
                    base_tag = tools.get_inverse_transform_object(tag_base)



                    cords_vec = tools.get_translation_vector(tools.get_transformation_matrix(base_tag))
                    xx = cords_vec.x
                    yy = cords_vec.y
                    zz = cords_vec.z
                    yaw = tools.get_euler_angles(base_tag.rotation)[2]
                    self.yaw = yaw
                    

                    if(abs(xx)<0.00001):
                        rospy.logerr("[AVG_POSITION_ANGLE] Division through zero is not allowed! xx:{0},y::{1},z:{2}".format(xx,yy,zz))
                    
                    else: 

                        alpha_dock = math.atan(yy/xx)

                        alpha_pos = self.get_direction_angle()



                        if(math.isfinite(alpha_pos)):
                            self.avg_pos.new_value(alpha_pos)
                            self.avg_dock.new_value(alpha_dock)
                            
                            self.avg_x.new_value(xx)
                            self.avg_y.new_value(yy)
                            self.avg_z.new_value(zz)


                            avg_pos_angle = self.avg_pos.avg()
                            avg_dock_angle = self.avg_dock.avg()
                           
                            avg_position_x = self.avg_x.avg()
                            avg_position_y = self.avg_y.avg()
                            avg_position_z = self.avg_z.avg()
                            
                            self.avg_position_angle = avg_pos_angle
                            self.avg_docking_angle = avg_dock_angle
                            
                            self.avg_position_x = avg_position_x
                            self.avg_position_y = avg_position_y
                            self.avg_position_z = avg_position_z

                else:
                    print("[AVG_POSITION_ANGLE] tag_{} not detected".format(id))
                    return
                
                
##################################################################
################## MOVEMENT FUNCTIONS ############################
##################################################################

#FUNDAMENTAL FUNC
    def drive_forward(self,distance):
        rospy.loginfo("[DRIVE_FORWARD] Start to move forward...")
        
        #-0.055 is a magic offset to drive exact distances  
        distance = distance - 0.055

        velocity = 0.15
        direction = distance / abs(distance)

        #We actually save the start Transformation
        startTransform = self.odom_transform
        startPos = tools.get_translation_vector(tools.get_transformation_matrix(startTransform))

        goal = distance
        driven_x = 0
        driven_y = 0
        pos_now = geometry_msgs.msg.Vector3()
        base = geometry_msgs.msg.Twist()
        while direction*goal <= direction * distance:
            base = geometry_msgs.msg.Twist()
            base.angular.z = 0
            base.linear.x = direction*velocity
            self.vel_pub.publish(base)

            #Calculate the driven way which is the difference between 
            #the startTransform and the actual transform
            pos_now = tools.get_translation_vector(tools.get_transformation_matrix(self.odom_transform))
            driven_x = pos_now.x - startPos.x
            driven_y = pos_now.y - startPos.y
            goal = direction * math.sqrt(driven_x*driven_x + driven_y*driven_y) 
            
            rospy.loginfo("[DRIVE_FORWARD] The driven distance is {}".format(goal))

            rospy.sleep(0.3)

        zero = geometry_msgs.msg.Twist()
        self.vel_pub.publish(zero)

#FUNDAMENTAL FUNC
    def drive_backward(self,distance):
        rospy.loginfo("[DRIVE_BACKWARDS]")
        self.drive_forward(-distance)

# with self.avg_docking_angle
    # def move_angle(self,alpha_rad):
    #     self.startReadingAngle()
    #     #params
    #     epsilon = 2
    #     angular_velocity = 0.1
    #     dt = 0.1

    #     rospy.loginfo("[MOVE-ANGLE] CHECK ==> abs(alpha_rad) : {}".format(abs(alpha_rad*(180/self.M_PI))))
    #     rospy.loginfo("[MOVE-ANGLE], CHECK ==>  abs(alpha_rad) - abs(self.avg_docking_angle): {0} - {1} = {2}".format(alpha_rad*(180/self.M_PI), self.avg_docking_angle*(180/self.M_PI),(abs(alpha_rad) - abs(self.avg_docking_angle))*(180/self.M_PI)))
         
    #     base = geometry_msgs.msg.Twist()
        
    #     if alpha_rad <0:
    #         angular_velocity = angular_velocity * -1
        
    #     while (180/self.M_PI)*(abs(alpha_rad) - abs(self.avg_docking_angle)) > epsilon:

    #         base.linear.x = 0
    #         base.angular.z = angular_velocity
    #         self.vel_pub.publish(base)
    #         rospy.sleep(dt)
    #         rospy.loginfo("[MOVE-ANGLE], CHECK ==>  abs(alpha_rad) - abs(self.avg_docking_angle): {0} - {1} = {2}".format(alpha_rad*(180/self.M_PI), self.avg_docking_angle*(180/self.M_PI),(abs(alpha_rad) - abs(self.avg_docking_angle))*(180/self.M_PI)))
    #         print()
        
    #     base.angular.z = 0
    #     self.vel_pub.publish(base)

    #     self.stopReadingAngle()

# with docking_angle()
    def move_angle(self,alpha_rad):
        self.startReadingAngle()
        #params
        epsilon = 2
        angular_velocity = 0.1
        dt = 0.1

        rospy.loginfo("[MOVE-ANGLE] CHECK ==> abs(alpha_rad) : {}".format(abs(alpha_rad*(180/self.M_PI))))
        rospy.loginfo("[MOVE-ANGLE], CHECK ==>  abs(alpha_rad) - abs(self.avg_docking_angle): {0} - {1} = {2}".format(alpha_rad*(180/self.M_PI), self.docking_angle()*(180/self.M_PI),(abs(alpha_rad) - abs(self.docking_angle()))*(180/self.M_PI)))
         
        base = geometry_msgs.msg.Twist()
        
        if alpha_rad <0:
            angular_velocity = angular_velocity * -1
        
        while (180/self.M_PI)*(abs(alpha_rad) - abs(self.docking_angle())) > epsilon:

            base.linear.x = 0
            base.angular.z = angular_velocity
            self.vel_pub.publish(base)
            rospy.sleep(dt)
            rospy.loginfo("[MOVE-ANGLE], CHECK ==>  abs(alpha_rad) - abs(self.avg_docking_angle): {0} - {1} = {2}".format(alpha_rad*(180/self.M_PI), self.docking_angle()*(180/self.M_PI),(abs(alpha_rad) - abs(self.docking_angle()))*(180/self.M_PI)))
            print()
        
        base.angular.z = 0
        self.vel_pub.publish(base)

        self.stopReadingAngle()


##################################################################
################## TAG DETECTION RELATED #########################
##################################################################    
    
    def searchTag(self):

        rospy.loginfo("[searchTag] Searching Tag...")
        self.find_tag = True # triggers to findTag() callback function
        rospy.sleep(1.5)
        base = geometry_msgs.msg.Twist()

        while not self.TAG_AVAILABLE:
            rospy.loginfo("[searchTag] Tag is not detected, robot will rotate")
            base.linear.x = 0.0
            rospy.loginfo("[searchTag] Check wheter robot rotates correctly")
            base.angular.z = 0.1
	
            self.vel_pub.publish(base)
            rospy.loginfo("[searchTag] Move to search")
        
        base.angular.z = 0
        self.vel_pub.publish(base)
        self.find_tag = False # switch off the callback function findTag

# with self.avg_docking_angle    
    # def watchTag(self):

    #     self.startReadingAngle()
        
    #     #params
    #     epsilon = 1
    #     angular_velocity = 0.1
    #     dt = 0.1

    #     rospy.loginfo("[WATCH-TAG], CHECK ==> self.avg_docking_angle : {}".format(abs(self.avg_docking_angle*(180/self.M_PI))))
    #     base = geometry_msgs.msg.Twist()

    #     if self.avg_docking_angle < 0:
    #         angular_velocity = angular_velocity * -1
    #     while abs((180/self.M_PI)*self.avg_docking_angle) > epsilon:
    #         base.linear.x = 0
    #         base.angular.z = angular_velocity
    #         self.vel_pub.publish(base)
    #         rospy.sleep(dt)
    #         rospy.loginfo("[WATCH-TAG], CHECK ==> self.avg_docking_angle : {}".format(abs(self.avg_docking_angle*(180/self.M_PI))))
    #         print()
    #     base.angular.z = 0
    #     base.linear.x = 0
    #     self.vel_pub.publish(base)

    #     self.stopReadingAngle()

# with docking_angle()    
    def watchTag(self):

        self.startReadingAngle()
        #params
        epsilon = 1
        angular_velocity = 0.1
        dt = 0.1

        rospy.loginfo("[WATCH-TAG], CHECK ==> self.avg_docking_angle : {}".format(abs(self.docking_angle()*(180/self.M_PI))))
        base = geometry_msgs.msg.Twist()

        if self.docking_angle() < 0:
            angular_velocity = angular_velocity * -1
        while abs((180/self.M_PI)*self.docking_angle()) > epsilon:
            base.linear.x = 0
            base.angular.z = angular_velocity
            self.vel_pub.publish(base)
            rospy.sleep(dt)
            rospy.loginfo("[WATCH-TAG], CHECK ==> self.avg_docking_angle : {}".format(abs(self.docking_angle()*(180/self.M_PI))))
            print()
        base.angular.z = 0
        base.linear.x = 0
        self.vel_pub.publish(base)

        self.stopReadingAngle()


##################################################################
###################### TOOL FUNCTIONS ############################
##################################################################       
    def startReadingAngle(self):
        self.start_avg = False
        self.avg_pos.flush_array()
        self.avg_dock.flush_array()
        self.avg_x.flush_array()
        self.avg_y.flush_array()
        self.start_avg = True
        rospy.sleep(3)
    
    def stopReadingAngle(self):
        self.start_avg = False

##################################################################
################## MAIN FUNCTIONS ################################
##################################################################


#MAIN FUNC 1
    # def positioning(self):
    #     self.startReadingAngle()
    #     a_pos_deg = self.avg_position_angle *(180/self.M_PI)

    #     pos = geometry_msgs.msg.Vector3()
    #     pos.x = self.avg_position_x
    #     pos.y = self.avg_position_y

    #     self.stopReadingAngle()

    #     rospy.loginfo("avg_position_angle: {}".format((self.avg_position_angle)*(180/self.M_PI)))


    #     epsilon = 5 # from watchTag() function
    #     if abs(a_pos_deg) < epsilon:
    #         rospy.loginfo("[POSITIONING] Start frontal docking without positioning")
    #         self.startReadingAngle()
    #         # self.docking()
        
    #     else:
    #         rospy.loginfo("[POSITIONING] Robot should position itself")

    #         if a_pos_deg < 0.0:
    #             rospy.loginfo("[POSITIONING] Robot should turn right")
    #         elif a_pos_deg > 0.0:
    #             rospy.loginfo("[POSITIONING] Robot should turn left")

    #     self.linearApproach()
    
    def positioning(self):
        a_pos_deg = self.get_direction_angle()*(180/self.M_PI)

        rospy.loginfo("position_angle: {}".format((self.get_direction_angle())*(180/self.M_PI)))

        epsilon = 2 # from watchTag() function
        if abs(a_pos_deg) < epsilon:
            rospy.loginfo("[POSITIONING] Start frontal docking without positioning")
            self.startReadingAngle()
            # self.docking()
        else:
            self.linearApproach()


#MAIN FUNC 2
    """
    LINEAR APPROACH
    """
    def linearApproach(self):
        alpha = self.get_direction_angle()

        #param
        distance = 30/100 #30 cm

        point = self.get_point_base_tag()
        x = point.x - distance
        y = point.y 
        y = self.initial_pos_y

        epsilon_deg = self.M_PI/2 - (math.atan(x/y) + self.get_direction_angle())

        way = math.sqrt(math.pow(x,2)+math.pow(y,2))

        if alpha < 0.0: # TURN RIGHT
            rospy.loginfo("[linearApproach] Turning right with epsilon={}".format(epsilon_deg))
            self.move_angle(-epsilon_deg)
            self.drive_forward(way)
            self.move_angle(epsilon_deg)
        
        if alpha > 0.0: #TURN LEFT
            rospy.loginfo("[linearApproach] Turning left with epsilon={}".format(epsilon_deg))
            self.move_angle(epsilon_deg)
            #self.drive_forward(way)
            #self.move_angle(-epsilon_deg)
        

        rospy.loginfo("[linearApproach] FINISHED")
        rospy.loginfo("[linearApproach] Start Docking!")
        
        self.startReadingAngle()
        #self.docking()


#MAIN FUNC 4
    """
    DOCKING STAGE
    """
    def docking(self):
        pass

    def check(self):
        self.startReadingAngle()
        while True:
            #1
            #check whether docking_angle() == self.avg_docking_angle 
            rospy.loginfo("[CHECK] self.docking_angle() : {}".format(self.docking_angle()))
            print()
            rospy.loginfo("[CHECK] self.avg_docking_angle : {}".format(self.avg_docking_angle))
            
            #2
            #check how pos.x & pos.y & pos.z of the transform from base to tag changes 
            # two variants: self.get_point_base_tag(), self.avg_docking_angle(self.avg_pos_x ...)
            # base = geometry_msgs.msg.Twist()
            # base.angular.z = 0.05
            # # base.linear.x = 0.05
            # self.vel_pub.publish(base)

            #2.1
            #self.get_point_base_tag()
            # point = self.get_point_base_tag()
            # rospy.loginfo("[CHECK] point.x: {}".format(point.x))
            # print()
            # rospy.loginfo("[CHECK] point.y: {}".format(point.y))
            # print()
            # rospy.loginfo("[CHECK] point.z: {}".format(point.z))
            # #2.2
            # #self.avg_pos_x, ...
            # rospy.loginfo("[CHECK] self.avg_pos_x: {}".format(self.avg_position_x))
            # print()
            # rospy.loginfo("[CHECK] self.avg_pos_y: {}".format(self.avg_position_y))
            # print()
            # rospy.loginfo("[CHECK] self.avg_pos_z: {}".format(self.avg_position_z))
            # print()
            
        
            #3
            #check how self.get_direction_angle() changes 
            # base = geometry_msgs.msg.Twist()
            # base.angular.z = 0.05
            # # base.linear.x = 0.05
            # self.vel_pub.publish(base)
            # rospy.loginfo("[CHECK] self.get_direction_angle() : {}".format(self.get_direction_angle()))
            # print()

            #4 modify the funcs according to the results of above tests


#MAIN FUNC 0-STARTER
    def startDocking(self):
        rospy.sleep(0.5)
        self.searchTag()
        rospy.loginfo("[START_DOCKING] after searchTag()")
        rospy.loginfo("[START_DOCKING] watchTag() starts")
        self.check()
        self.watchTag()
        rospy.loginfo("[START_DOCKING]] watchTag() finished")

        
        rospy.loginfo("[POSITIONING] Starting...")
        self.positioning()
        rospy.loginfo("[START_DOCKING] after positioning()")
       
        """"
        #self.adjusting()
        #rospy.loginfo("[START_DOCKING] after adjusting()")
        #rospy.sleep(2)
        #self.startReadingAngle()
        #self.docking()
        #rospy.loginfo("[START_DOCKING] after docking()")
        
        #TAKES 5 SECS TO REACH self.docking()

        """


def main():
    tp = Docking()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

if __name__ == "__main__":
    main()


