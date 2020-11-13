#!/usr/bin/python
import numpy as np
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from cv_bridge import CvBridge
from pepper_msgs.msg import ImageWithDirection


class CameraNode(NaoqiNode):

  __slots__ = 'videoDeviceProxy', 'videoDevice', '_pub'

  def __init__(self):
    NaoqiNode.__init__(self, 'camera_node')
    self.connectNaoQi()
    cameraID = 0  # CameraID 0 means TopCamera
    resolution = 3  # Resolution 3 means Image of 1280*960px
    colorSpace = 13  # Color Space 13 means BGR
    self.videoDevice = self.videoDeviceProxy.subscribeCamera("pepper_top_camera", cameraID, resolution, colorSpace)

  def connectNaoQi(self):
    rospy.loginfo("CameraNode connecting to NaoQi at %s:%d", self.pip, self.pport)
    self.videoDeviceProxy = self.get_proxy("ALVideoDevice")
    if self.videoDeviceProxy is None:
      exit(1)
  
  def start(self):
    rospy.Subscriber(rospy.get_param("take_picture_topic"), int, self.take_picture_cb)
    self._pub = rospy.Publisher(rospy.get_param('image_topic'), ImageWithDirection, queue_size=0)

  def take_picture_cb(self, direction):
    result = self.videoDeviceProxy.getImageRemote(self.videoDevice)
    if result == None:
      return None
    if result[6] == None:
      return None
    width = result[0]
    height = result[1]
    image = np.zeros((height, width), np.uint8)
    values = map(ord, list(result[6]))
    i = 0
    for x in range(width):
      for y in range(height):
        for z in range(3):
          image.itemset((y, x, z), values[i + z])
        i += 3
    image = CvBridge().cv2_to_imgmsg(image)
    message = ImageWithDirection()
    message.image = image
    message.direction = direction
    self._pub.publish(message)
    

def main():
  CameraNode().start()
  try:
    rospy.spin()
  except (KeyboardInterrupt, rospy.exceptions) as e:
    rospy.loginfo("shutdown: %s" % e)


if __name__ == "__main__":
  main()
