#!/usr/bin/env python3
import rospy
import tools

from sensor_msgs.msg import Imu
class IMU():

    def __init__(self):
        rospy.init_node("IMU Reciever")
        
        self.imu_sub = rospy.Subscriber("imu/data",Imu,self.callback)

    def callback(self,data):
        quat = data.orientation
        euler = tools.get_euler_angles(quat)
        yaw = euler[2]
        rospy.loginfo("Yaw angle: {}".format(yaw))
def main():
    tp = IMU()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
