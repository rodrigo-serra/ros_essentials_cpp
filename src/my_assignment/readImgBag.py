#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from imgProcessing import detectLine

class ReadImgFromBag:
  def __init__(self, topic_name):
    self.bridge = CvBridge()
    self.sub = rospy.Subscriber(topic_name, Image, self.imgCallback)

  def imgCallback(self, rosImg):
    try:
      cvImg = self.bridge.imgmsg_to_cv2(rosImg, "bgr8")
    except CvBridgeError as error:
      rospy.logerr(error)

    endImg = detectLine(cvImg)

    cv2.imshow("Image window", endImg)
    cv2.waitKey(3)
  
  
def main():
  rospy.init_node('read_img_bag', anonymous=True)
  streamer = ReadImgFromBag('camera/color/image_raw')
  # streamer = ReadImgFromBag('/fiducial_images/compressed')
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting Down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()