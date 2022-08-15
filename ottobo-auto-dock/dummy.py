#!/usr/bin/env python3
import rospy
import tools
import apriltag_ros.msg
import math
import geometry_msgs.msg
from sensor_msgs.msg import Imu

class IMU():

    def __init__(self):
        rospy.init_node("IMU Reciever")
        self.imu_sub = rospy.Subscriber("camera/imu",Imu,self.callback)
        self.vel_pub = rospy.Publisher("cmd_vel",geometry_msgs.msg.Twist,queue_size=10)


    def callback(self,data):
        base = geometry_msgs.msg.Twist()
        quat = data.orientation
        euler = tools.get_euler_angles(quat) 
        yaw = euler[2]

        if yaw < 0:
            yaw += 2*math.pi
        
        rospy.loginfo("Yaw angle: {}".format(yaw*(180/math.pi)))
        
        # base.angular.z = 0.05
        # self.vel_pub.publish(base)



        
def main():
    tp = IMU()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
