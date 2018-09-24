#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    rospy.init_node('image_publisher', anonymous=True)
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()

    cv_image_orig = cv2.imread("i3_top.jpg")

    b_channel, g_channel, r_channel = cv2.split(cv_image_orig)
    alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 250 #creating a dummy alpha channel image.
    cv_image = cv2.merge([b_channel, g_channel, r_channel, alpha_channel], 4)

    cv_image[:][0:450] = [0, 0, 0, 50]

    # cv2.imwrite("out.png", cv_image)

    r = rospy.Rate(10)

    while(not rospy.is_shutdown()):
        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgra8"))
        except CvBridgeError as e:
          print(e)

        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)

        r.sleep()

def main():
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
