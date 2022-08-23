#!/usr/bin/env python3
import math
from threading import Thread
from avg import avg
import rospy
import geometry_msgs.msg
import nav_msgs.msg
import apriltag_ros.msg
import tf
import tools
import math

class Docking():

    def __init__(self):
        rospy.init_node("Docking")
        #to control motors
        self.vel_pub = rospy.Publisher("cmd_vel",geometry_msgs.msg.Twist,queue_size=10)

        rospy.loginfo("DOCKING INIT\n")
        rospy.sleep(1) #in source code, its purpose is said to be for waiting for the AprilTagDetectionNode
        
        #TF listener to listen frames
        self.tf_listener = tf.TransformListener()
        
        self.base_link = "base_link"
        self.camera_link = "camera_link"
        self.optical_frame = "camera_color_optical_frame"
        self.approach_frame = "approachFrame"
        self.odom_frame = "odom"

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
        rospy.loginfo("[INIT] CameraLink -> baseLink\n")
        self.get_optical_frame()
        rospy.loginfo("[INIT] optical -> camera\n")

        #is a bool to start and stop get_avg_position_callback
        self.start_avg = False 

        #is a bool to start and stop findTag callback
        self.find_tag = False

        #used in findTag callback
        self.TAG_AVAILABLE = False

        #to keep trying after failed attempts
        self.try_more = True

        #Average of several vars
        #NEED MORE EXPLANATION
        #params
        rospy.loginfo("[INIT] avg inits\n")
        self.avg_pos = avg(10)
        self.avg_dock = avg(2)
        self.avg_x = avg(10)
        self.avg_y = avg(10)
        self.avg_z = avg(10)

        self.avg_position_angle = 0.0
        self.avg_docking_angle = 0.0

        self.avg_position_x = 0.0
        self.avg_position_y = 0.0
        self.avg_position_z = 0.0

        self.yaw = 0


        self.docking_status = True

        self.M_PI = math.pi #180 degrees, pi in radians i suppose

        self.odom_pos = geometry_msgs.msg.Vector3()


        #tag_sub is for getting average position angle
        self.tag_sub = rospy.Subscriber("tag_detections",apriltag_ros.msg.AprilTagDetectionArray,self.get_avg_position_angle_callback)  

        #find_tag_sub is to search for specific tag_id 
        self.find_tag_sub = rospy.Subscriber("tag_detections",apriltag_ros.msg.AprilTagDetectionArray,self.findTag)


        #odom_sub is to get /odom frame
        self.odom_sub = rospy.Subscriber("odom",nav_msgs.msg.Odometry,self.get_odom_pos)

        self.broadcast_approach = False

        rospy.loginfo("[INIT] Start Docking Process\n")
        self.startDocking()

##################################################################
################## INIT OF LINKS #################################
##################################################################

    def get_cameraLink_baseLink(self):
        """""
        initialize transform_cam_base 
        Camera_link->base_link 
        
        """""
        try:
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
            self.tf_listener.waitForTransform(self.optical_frame,self.camera_link,rospy.Time(0),rospy.Duration(5.0))
            pos, quad =self.tf_listener.lookupTransform(self.optical_frame,self.camera_link,rospy.Time(0))
            pos_obj = geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2])
            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            self.transform_optical_cam = geometry_msgs.msg.Transform(pos_obj,quad_obj)
        except Exception as ex:
            rospy.logerr(ex)  
            rospy.sleep(1.0)


##################################################################
############ CALCULATING ANGLES & RELATIVE POINTS ################
##################################################################
    """
    CALCULATING DOCKING_ANGLE
    """
    def docking_angle(self):

        transform = geometry_msgs.msg.Transform()
        
        try:
            self.tf_listener.waitForTransform(self.base_link,self.tag_id,rospy.Time(0),rospy.Duration(5.0))
            pos, quad = self.tf_listener.lookupTransform(self.base_link,self.tag_id,rospy.Time(0))
            pos_obj = geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2])
            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            transform = geometry_msgs.msg.Transform(pos_obj,quad_obj)
        
        except Exception as e:
            rospy.logerr(f"docking_angle() {e}\n")
            rospy.sleep(1)
        
        tags_vec = tools.get_translation_vector(tools.get_transformation_matrix(transform))
        
        map_tag_x = tags_vec.x
        map_tag_y = tags_vec.y
        map_tag_z = tags_vec.z

        return math.atan(map_tag_y/map_tag_x)  

    """
    CALCULATING DIRECTION ANGLE
    """

    def get_direction_angle(self):
        transform = geometry_msgs.msg.TransformStamped()

        try:
            self.tf_listener.waitForTransform(self.tag_id,self.base_link,rospy.Time(0),rospy.Duration(5.0))
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
        
        yaw = tools.get_euler_angles(geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3]))

        wd_rad = math.atan(tag_x/tag_y)

        return yaw[2]
    
    """
    CALCULATING APPROACH FRAME ANGLE
    """
    def approach_angle(self):
        #take the pos & quad of approachTarget frame relative to base tag
        try:
            self.tf_listener.waitForTransform(self.base_link,self.approach_frame,rospy.Time(0),rospy.Duration(5.0))
            pos, quad = self.tf_listener.lookupTransform(self.base_link,self.approach_frame,rospy.Time(0))
            pos_obj = geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2])
            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            transform = geometry_msgs.msg.Transform(pos_obj,quad_obj)
        
        except Exception as e:
            rospy.logerr(f"approach_angle() {e}")
            rospy.sleep(1)
            return
        
        approach_vec = tools.get_translation_vector(tools.get_transformation_matrix(transform))

        y = approach_vec.y
        x = approach_vec.x 
    
        return math.atan(y/x)
   
    """
    CALCULATING POINT RELATIVE TO BASE_TAG
    """
    def get_point_odom_tag(self):

        transform = geometry_msgs.msg.Transform()
        
        try:
            self.tf_listener.waitForTransform(self.odom_frame,self.tag_id,rospy.Time(0),rospy.Duration(5.0))
            pos, quad = self.tf_listener.lookupTransform(self.odom_frame,self.tag_id,rospy.Time(0))
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
    

    def get_point_approach_base(self):
            #take the pos & quad of approachTarget frame relative to base tag
        try:
            self.tf_listener.waitForTransform(self.approach_frame,self.base_link,rospy.Time(0),rospy.Duration(5.0))
            pos, quad = self.tf_listener.lookupTransform(self.approach_frame,self.base_link,rospy.Time(0))
            pos_obj = geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2])
            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            transform = geometry_msgs.msg.Transform(pos_obj,quad_obj)
        
        except Exception as e:
            rospy.logerr(f"get_point_approach_base {e}")
            rospy.sleep(1)
            return
        approach_vec = tools.get_translation_vector(tools.get_transformation_matrix(transform))

        y = approach_vec.y
        x = approach_vec.x
        z = approach_vec.z 
        
        return geometry_msgs.msg.Point(x,y,z)

    def get_point_base_approach(self):
            #take the pos & quad of approachTarget frame relative to base tag
        try:
            self.tf_listener.waitForTransform(self.base_link,self.approach_frame,rospy.Time(0),rospy.Duration(5.0))
            pos, quad = self.tf_listener.lookupTransform(self.base_link,self.approach_frame,rospy.Time(0))
            pos_obj = geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2])
            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            transform = geometry_msgs.msg.Transform(pos_obj,quad_obj)
        
        except Exception as e:
            rospy.logerr(f"get_point_approach_base {e}")
            rospy.sleep(1)
            return
        approach_vec = tools.get_translation_vector(tools.get_transformation_matrix(transform))

        y = approach_vec.y
        x = approach_vec.x
        z = approach_vec.z 
        
        return geometry_msgs.msg.Point(x,y,z)
    """
    CALCULATING QUATERNION RELATIVE TO BASE_TAG
    """
    def get_quat_odom_tag(self):  
        transform = geometry_msgs.msg.Transform()
        
        try:
            self.tf_listener.waitForTransform(self.odom_frame,self.tag_id,rospy.Time(0),rospy.Duration(5.0))
            pos, quad = self.tf_listener.lookupTransform(self.odom_frame,self.tag_id,rospy.Time(0))
            pos_obj = geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2])
            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            transform = geometry_msgs.msg.Transform(pos_obj,quad_obj)
        
        except Exception as e:
            rospy.logerr(f"docking_angle() {e}")
            rospy.sleep(1)

        return quad_obj


##################################################################
################## CALLBACK FUNCTIONS ############################
##################################################################

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
                        rospy.loginfo("Tag is avaliable\n")

    """
    INIT OF self.odom_transform
    """
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

    def get_avg_position_angle_callback(self,data):
        """
        GETS THE AVERAGE POSITION
        """
        if self.broadcast_approach:
            self.broadcast_approachFrame()
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
                    rospy.logerr("[AVG_POSITION_ANGLE] tag_{} not detected\n".format(id))
                    return                
##################################################################
################## MOVEMENT FUNCTIONS ############################
##################################################################

    def drive_forward(self,distance):
        rospy.loginfo("[DRIVE_FORWARD] Start to move forward...\n")
          
        distance = distance - 0.055

        velocity = 0.15
        direction = distance / abs(distance)

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
            
            rospy.loginfo("[DRIVE_FORWARD] The driven distance is {}\n".format(goal))

            rospy.sleep(0.3)

        self.vel_pub.publish( geometry_msgs.msg.Twist())


##################################################################
################## TAG DETECTION RELATED #########################
##################################################################    
    
    def searchTag(self):

        rospy.loginfo("[searchTag] Searching Tag...")
        self.find_tag = True # triggers to findTag() callback function
        rospy.sleep(1.5)
        base = geometry_msgs.msg.Twist()

        while not self.TAG_AVAILABLE:
            rospy.loginfo("[searchTag] Tag is not detected, robot will rotate to left\n")
            base.linear.x = 0.0
            rospy.loginfo("[searchTag] Check wheter robot rotates correctly\n")
            base.angular.z = 0.1
	
            self.vel_pub.publish(base)
            rospy.loginfo("[searchTag] Move to search\n")
        
        base.angular.z = 0
        self.vel_pub.publish(base)
        self.find_tag = False # switch off the callback function findTag


    def watchTag(self):
        #params
        epsilon = 4
        angular_velocity = 0.1
        dt = 0.1

        rospy.loginfo("[WATCH-TAG], CHECK ==> docking_angle() : {}\n".format(abs(self.docking_angle()*(180/self.M_PI))))
        base = geometry_msgs.msg.Twist()

        if self.docking_angle() < 0:
            angular_velocity = angular_velocity * -1
        while abs((180/self.M_PI)*self.docking_angle()) > epsilon:
            base.linear.x = 0
            base.angular.z = angular_velocity
            self.vel_pub.publish(base)
            rospy.sleep(dt)
            rospy.loginfo("[WATCH-TAG], CHECK ==> docking_angle() : {}\n".format(abs(self.docking_angle()*(180/self.M_PI))))
        base.angular.z = 0
        base.linear.x = 0
        self.vel_pub.publish(base)


    def watchApproachFrame(self):
        #params
        epsilon = 4
        angular_velocity = 0.1
        dt = 0.1

        rospy.loginfo("[WATCH-APPROACH_FRAME], approach_angle() : {}\n".format(abs(self.approach_angle()*(180/self.M_PI))))
        base = geometry_msgs.msg.Twist()

        if self.approach_angle() < 0:
            angular_velocity = angular_velocity * -1
       
        while abs((180/self.M_PI)*self.approach_angle()) > epsilon:
            base.linear.x = 0
            base.angular.z = angular_velocity
            self.vel_pub.publish(base)
            rospy.sleep(dt)
            rospy.loginfo("[WATCH-APPROACH_FRAME], approach_angle() : {}\n".format(abs(self.approach_angle()*(180/self.M_PI))))
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
    
    def positioning(self):
        self.startReadingAngle()
        # a_pos_deg = self.get_direction_angle()*(180/self.M_PI)
        a_pos_deg = self.avg_position_angle*(180/self.M_PI)

        rospy.loginfo("position_angle: {}\n".format((self.avg_position_angle)*(180/self.M_PI)))

        epsilon = 2 # from watchTag() function
        if abs(a_pos_deg) < epsilon:
            rospy.loginfo("[POSITIONING] Robot is located in front of docking station. No positioning needed\n")
            rospy.loginfo("[POSITIONING] Start Docking!\n")
            self.docking()
        else:
            self.broadcast_approach = True
            self.linearApproach()
            #self.broadcast_approach = False

        self.stopReadingAngle()

#MAIN FUNC 2
    #Thread
    def broadcast_approachFrame(self):
        rospy.loginfo("[BROADCAST_APPROACH_FRAME]")
        #param
        distance = 75/100 #75 cm

        #determine which translation vector & quat to use  base_tag or tag_base
        point = self.get_point_odom_tag()
        quat = self.get_quat_odom_tag()
        #assign new x value
        point.x -= distance 
        while self.broadcast_approach:
            br = tf.TransformBroadcaster()
            br.sendTransform([point.x,point.y,point.z],[quat.x,quat.y,quat.z,quat.w],rospy.Time.now(),self.approach_frame,self.odom_frame)
    """
    LINEAR APPROACH
    """
    def linearApproach(self):
        x = self.get_point_approach_base().z
        # x = self.get_point_base_approach().x
        y = self.get_point_approach_base().x
        # y = self.get_point_base_approach().y



        #check whether way is correct
        
        self.watchApproachFrame()
        
        #Try below as well
        # x = self.get_point_approach_base().z
        # y = self.get_point_approach_base().x
        way = math.sqrt((x*x)+(y*y))
        rospy.loginfo("way : {}".format(way))
        
        self.broadcast_approach = False
        self.drive_forward(way)
        rospy.loginfo("[linearApproach] FINISHED\n")
        
#MAIN FUNC 4
    """
    DOCKING STAGE
    """
    def docking(self):
        rospy.loginfo("[DOCKING] Docking is starting\n")


#MAIN FUNC 0-STARTER
    def startDocking(self):
        rospy.sleep(0.5)
        rospy.loginfo("[START_DOCKING] searchTag() starts...\n")
        self.searchTag()
        rospy.loginfo("[START_DOCKING] watchTag() starts...\n")
        self.watchTag()

        rospy.loginfo("[START_DOCKING] positioning() starts...\n")
        self.positioning()

        # rospy.loginfo("[START_DOCKING] Verify the tag is still visible\n")
        # rospy.loginfo("[START_DOCKING] searchTag() starts...\n")
        # self.searchTag()
        # rospy.loginfo("[START_DOCKING] Align the robot with docking station\n")
        # rospy.loginfo("[START_DOCKING] watchTag() starts...\n")
        # self.watchTag()
        # rospy.loginfo("[START_DOCKING] docking() starts...\n")
        # self.docking()




def main():
    tp = Docking()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

if __name__ == "__main__":
    main()


