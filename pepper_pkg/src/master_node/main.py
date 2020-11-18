#!/usr/bin/python
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_msgs.srv import LookAt
from pepper_msgs.srv import TakePicture
from std_msgs.msg import Int8


class MasterNode(NaoqiNode):

  __slots__ = 'motionProxy'

  def __init__(self):
    NaoqiNode.__init__(self, 'master_node')
    self.connectNaoQi()

  def connectNaoQi(self):
    self.pip = rospy.get_param('pip')
    self.pport = rospy.get_param('pport')
    rospy.loginfo("MasterNode connecting to NaoQi at %s:%d", self.pip, self.pport)
    self.motionProxy = self.get_proxy("ALRobotPosture")
    if self.motionProxy is None:
      exit(1)
    rospy.loginfo("ALRobotPosture successful!")
      
  def start(self):
    self.motionProxy.goToPosture("StandInit", 1.0)
    for direction in [1, 0, -1]:
      # Look at right, front, left
      ret = self._look_at(direction)
      if not ret:
        exit(1)
      # Take picture
      ret = self._take_picture(direction)
      if not ret:
        exit(1)
      # Wait
      rospy.sleep(rospy.Duration(1.0))
    self.motionProxy.goToPosture("StandInit", 1.0)

  def _look_at(self, direction):
    service = rospy.get_param('look_at_service')
    rospy.wait_for_service(service)
    try:
      look_at = rospy.ServiceProxy(service, LookAt)
      return look_at(direction).ready
    except rospy.ServiceException as e:
      print("Service call failed: %s" % e)
      return False

  def _take_picture(self, direction):
    service = rospy.get_param('take_picture_service')
    rospy.wait_for_service(service)
    try:
      take_picture = rospy.ServiceProxy(service, TakePicture)
      return take_picture(direction).ready
    except rospy.ServiceException as e:
      print("Service call failed: %s" % e)
      return False


def main():
  MasterNode().start()
  try:
    rospy.spin()
  except (KeyboardInterrupt, rospy.exceptions) as e:
    rospy.loginfo("shutdown: %s" % e)


if __name__ == "__main__":
  main()
