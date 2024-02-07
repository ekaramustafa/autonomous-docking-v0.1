#!/usr/bin/env python3
import rospy
from threading import Thread,Lock
import geometry_msgs.msg
from sensor_msgs.msg import Imu
import std_msgs.msg 

class dummy():

    def __init__(self):
        rospy.init_node("dummy")
        self.str_sub = rospy.Subscriber("dummy",std_msgs.msg.String,self.callback)
        self.vel_pub = rospy.Publisher("cmd_vel",geometry_msgs.msg.Twist,queue_size=10)
        self.mutex = Lock()
        self.controller = True

    def callback(self,data):
        rospy.loginfo("{}\n".format(data))
        self.callthread()
    
    def printer(self):
        rospy.loginfo("Thread thread\n")

    def callthread(self):
        t1 = Thread(self.printer())
        t1.start()
def main():
    tp = dummy()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
