#!/usr/bin/python
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_msgs.msg import ImageWithDirection
from pepper_msgs.srv import TakePicture, TakePictureResponse
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import genpy


class CameraNode(NaoqiNode):

  __slots__ = 'videoDeviceProxy', 'videoDevice', '_pub'

  def __init__(self):
    NaoqiNode.__init__(self, 'camera_node')
    self.connectNaoQi()
    cameraID = 0  # CameraID 0 means TopCamera
    resolution = 3  # Resolution 3 means Image of 1280*960px
    colorSpace = 13  # Color Space 13 means BGR
    self.videoDevice = self.videoDeviceProxy.subscribeCamera("pepper_top_camera", cameraID, resolution, colorSpace, 10)

  def connectNaoQi(self):
    self.pip = rospy.get_param('pip')
    self.pport = rospy.get_param('pport')
    rospy.loginfo("CameraNode connecting to NaoQi at %s:%d", self.pip, self.pport)
    self.videoDeviceProxy = self.get_proxy("ALVideoDevice")
    if self.videoDeviceProxy is None:
      exit(1)
    rospy.loginfo("ALVideoDevice successful!")
  
  def start(self):
    rospy.Service(rospy.get_param('take_picture_service'), TakePicture, self.handle_take_picture)
    self._pub = rospy.Publisher(rospy.get_param('image_topic'), ImageWithDirection, queue_size=0, latch=True)

  def handle_take_picture(self, req):
    direction = req.direction
    result = self.videoDeviceProxy.getImageRemote(self.videoDevice)
    if result == None or result[6] == None:
      rospy.loginfo("Problem!")
      return TakePictureResponse(False)
    message = ImageWithDirection()
    message.image = Image(
      header = Header(stamp=genpy.Time(secs=result[4], nsecs=1000*result[5])),
      height = result[1],
      width = result[0],
      encoding = "bgr8",
      is_bigendian = False,
      step = result[0] * result[2],
      data = result[6]
    )
    message.direction = direction
    self._pub.publish(message)
    rospy.loginfo("image for direction {} ready".format(direction))
    return TakePictureResponse(True)
    

def main():
  CameraNode().start()
  try:
    rospy.spin()
  except (KeyboardInterrupt, rospy.exceptions) as e:
    rospy.loginfo("shutdown: %s" % e)


if __name__ == "__main__":
  main()
