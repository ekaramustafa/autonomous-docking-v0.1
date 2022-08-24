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
        #waits for AprilTag Node
        rospy.sleep(1)
        
        #TF listener to listen frames
        self.tf_listener = tf.TransformListener()
        
        self.base_link = "base_link"
        self.odom_frame = "odom"
        self.approach_frame = "approachFrame"
        self.docking_frame = "chargingStation"

        self.odom_transform = geometry_msgs.msg.Transform()

        #param
        self.tag_id = "tag_0"

        #directionangle_and_broadcast_callback controllers
        self.start_avg = False 

        #is a bool to start and stop findTag callback
        self.find_tag = False

        #used in findTag callback
        self.TAG_AVAILABLE = False

        #params
        rospy.loginfo("[INIT] Avg inits\n")
        self.avg_pos = avg(10)
        self.direction_angle = 0.0

        self.M_PI = math.pi

        self.odom_pos = geometry_msgs.msg.Vector3()

        self.tag_sub = rospy.Subscriber("tag_detections",apriltag_ros.msg.AprilTagDetectionArray,self.directionangle_and_broadcast_callback)  

        self.find_tag_sub = rospy.Subscriber("tag_detections",apriltag_ros.msg.AprilTagDetectionArray,self.findTag)

        self.odom_sub = rospy.Subscriber("odom",nav_msgs.msg.Odometry,self.get_odom_pos)

        #Broadcast controllers
        self.broadcast_approach = False

        #DOCKING
        #battery sub & callback which assign self.docking_status True
        self.docking_status = False

        rospy.loginfo("[INIT] Start Docking Process\n")
        self.startDocking()


##################################################################
################### CALCULATING ANGLES  ##########################
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

    def direction_angle(self):
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

##################################################################
################ CALCULATING RELATIVE POINTS #####################
##################################################################   
   
    """
    CALCULATING TAG'S POINT FROM ODOM TO TAG
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
            rospy.logerr(f"get_point_odom_tag() {e}")
            rospy.sleep(1)
        
        tags_vec = tools.get_translation_vector(tools.get_transformation_matrix(transform))
        
        map_tag_x = tags_vec.x
        map_tag_y = tags_vec.y
        map_tag_z = tags_vec.z
        
        return geometry_msgs.msg.Point(map_tag_x,map_tag_y,map_tag_z)

    """
    CALCULATING TAG'S QUATERNION FROM ODOM TO TAG
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
    """
    CALCULATING TRANSFORM FROM ODOM TO DOCKING
    """
    def get_transform_odom_docking(self):

        transform = geometry_msgs.msg.Transform()
        
        try:
            self.tf_listener.waitForTransform(self.odom_frame,self.docking_frame,rospy.Time(0),rospy.Duration(5.0))
            pos, quad = self.tf_listener.lookupTransform(self.odom_frame,self.docking_frame,rospy.Time(0))
            pos_obj = geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2])
            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            transform = geometry_msgs.msg.Transform(pos_obj,quad_obj)
        
        except Exception as e:
            rospy.logerr(f"get_transform_odom_docking() {e}\n")
            rospy.sleep(1)
        

        return transform
    """
    CALCULATING POINT FROM ODOM TO APPROACH
    """
    def get_point_odom_approach(self):
        #take the pos & quad of approachTarget frame relative to odom
        try:
            self.tf_listener.waitForTransform(self.odom_frame,self.approach_frame,rospy.Time(0),rospy.Duration(5.0))
            pos, quad = self.tf_listener.lookupTransform(self.odom_frame,self.approach_frame,rospy.Time(0))
            pos_obj = geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2])
            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            transform = geometry_msgs.msg.Transform(pos_obj,quad_obj)
        
        except Exception as e:
            rospy.logerr(f"get_point_odom_appraoch {e}")
            rospy.sleep(1)
            return
        vec = tools.get_translation_vector(tools.get_transformation_matrix(transform))

        y = vec.y
        x = vec.x
        z = vec.z 
        
        return geometry_msgs.msg.Point(x,y,z)
    """
    CALCULATING POINT FROM BASE TO APPROACH
    """
    def get_point_base_approach(self):
        #take the pos & quad of approachFrame relative to base_link
        try:
            self.tf_listener.waitForTransform(self.base_link,self.approach_frame,rospy.Time(0),rospy.Duration(5.0))
            pos, quad = self.tf_listener.lookupTransform(self.base_link,self.approach_frame,rospy.Time(0))
            pos_obj = geometry_msgs.msg.Vector3(pos[0],pos[1],pos[2])
            quad_obj = geometry_msgs.msg.Quaternion(quad[0],quad[1],quad[2],quad[3])
            transform = geometry_msgs.msg.Transform(pos_obj,quad_obj)
        
        except Exception as e:
            rospy.logerr(f"get_point_base_approach {e}")
            rospy.sleep(1)
            return
        approach_vec = tools.get_translation_vector(tools.get_transformation_matrix(transform))

        y = approach_vec.y
        x = approach_vec.x
        z = approach_vec.z 
        
        return geometry_msgs.msg.Point(x,y,z)
    
##################################################################
################## CALLBACK FUNCTIONS ############################
##################################################################
    """
    SEARCH FOR AVAILABLE TAG
    """
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

    def directionangle_and_broadcast_callback(self,data):
        """
        GETS THE AVERAGE POSITION
        """
        if self.broadcast_approach:
            self.broadcast_approachFrame()
        if self.start_avg:
            for i in range(len(data.detections)):
                id, = data.detections[i].id
                if self.tag_id == "tag_{}".format(id): 
                    alpha_pos = self.direction_angle()

                    if(math.isfinite(alpha_pos)):
                        self.avg_pos.new_value(alpha_pos)

                        avg_pos_angle = self.avg_pos.avg()
                        self.direction_angle = avg_pos_angle
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
################## TAG & FRAME RELATED ###########################
##################################################################    
    
    def searchTag(self):

        rospy.loginfo("[searchTag] Searching Tag...")
        self.find_tag = True # triggers to findTag() callback function
        rospy.sleep(1)
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

        rospy.loginfo("[WATCH-TAG], docking_angle() : {}\n".format(abs(self.docking_angle()*(180/self.M_PI))))
        base = geometry_msgs.msg.Twist()

        if self.docking_angle() < 0:
            angular_velocity = angular_velocity * -1
        while abs((180/self.M_PI)*self.docking_angle()) > epsilon:
            base.linear.x = 0
            base.angular.z = angular_velocity
            self.vel_pub.publish(base)
            rospy.sleep(dt)
            rospy.loginfo("[WATCH-TAG], docking_angle() : {}\n".format(abs(self.docking_angle()*(180/self.M_PI))))
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
################# CALLBACK CONTROLLERS ###########################
##################################################################       
    def startReadingAngle(self):
        self.start_avg = False
        self.avg_pos.flush_array()
        self.start_avg = True
        rospy.sleep(1)
    
    def stopReadingAngle(self):
        self.start_avg = False
##################################################################
################# BRODCAST FUNCTIONS #############################
################################################################## 
    def broadcast_approachFrame(self):
        #param
        distance = 100/100 #100 cm

        point = self.get_point_odom_tag()
        quat = self.get_quat_odom_tag()
        #assign new x value
        point.x -= distance 
        br = tf.TransformBroadcaster()
        while self.broadcast_approach:
            br.sendTransform([point.x,point.y,point.z],[quat.x,quat.y,quat.z,quat.w],rospy.Time.now(),self.approach_frame,self.odom_frame)
 
##################################################################
################## MAIN FUNCTIONS ################################
################################################################## 

# MAIN FUNC 1   
    def positioning(self):
        self.startReadingAngle()
        a_pos_deg = self.direction_angle*(180/self.M_PI)

        rospy.loginfo("[POSITIONING] Position_angle: {}\n".format((self.direction_angle)*(180/self.M_PI)))

        epsilon = 2
        if abs(a_pos_deg) < epsilon:
            rospy.loginfo("[POSITIONING] Robot is located in front of docking station. No positioning needed\n")
            rospy.loginfo("[POSITIONING] Start Docking!\n")
            self.frontal_docking()
        else:
            self.linearApproach()

        self.stopReadingAngle()

# MAIN FUNC 2   
    def linearApproach(self):
        #Broadcast approachFrame
        self.broadcast_approach = True
        
        x = self.get_point_base_approach().x  
        #x = self.get_point_odom_approach().x 
        #x -= abs(self.get_point_odom_base().x)    
        
        y = self.get_point_base_approach().y
        #y = self.get_point_odom_approach().y
        #y -= abs(self.get_point_odom_base().y)
        
        self.watchApproachFrame()
        rospy.sleep(1)

        way = math.sqrt((x*x)+(y*y))
        rospy.loginfo("way : {}".format(way))

        self.drive_forward(way)
        rospy.loginfo("[linearApproach] FINISHED\n")
        self.broadcast_approach = False
     
#MAIN FUNC 3
    def frontal_docking(self):
        rospy.loginfo("Docking is starting")
        pass


#STARTER FUNC
    def startDocking(self):
        rospy.sleep(0.5)
        
        rospy.loginfo("[START_DOCKING] searchTag() starts...\n")
        self.searchTag()
        
        rospy.loginfo("[START_DOCKING] watchTag() starts...\n")
        self.watchTag()

        rospy.loginfo("[START_DOCKING] positioning() starts...\n")
        self.positioning()
        
        # rospy.sleep(1.0)
        # rospy.loginfo("[START_DOCKING] Verify the tag is still visible\n")
        # rospy.loginfo("[START_DOCKING] searchTag() starts...\n")
        # self.searchTag()
        # rospy.loginfo("[START_DOCKING] Align the robot with docking station\n")
        # rospy.loginfo("[START_DOCKING] watchTag() starts...\n")
        # self.watchTag()
        # rospy.loginfo("[START_DOCKING] docking() starts...\n")
        # self.frontal_docking()

def main():
    doc = Docking()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

if __name__ == "__main__":
    main()


