#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera/color/image_raw",Image,self.callback)

    self.image_pub = rospy.Publisher("camera/flipped/image_raw",Image,queue_size=1)


    
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    img = cv2.rotate(cv_image, cv2.ROTATE_180)
###
##publishing the cv_image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('flip_camera', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)