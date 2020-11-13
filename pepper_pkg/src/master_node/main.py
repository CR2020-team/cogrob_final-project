#!/usr/bin/python
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_msgs.srv import LookAt

import sys
import rospy
from beginner_tutorials.srv import *


class MasterNode(NaoqiNode):

  __slots__ = 'motionProxy'

  def __init__(self):
    NaoqiNode.__init__(self, 'master_node')
    self.connectNaoQi()

  def connectNaoQi(self):
    rospy.loginfo("MasterNode connecting to NaoQi at %s:%d", self.pip, self.pport)
    self.motionProxy = self.get_proxy("ALMotion")
    if self.motionProxy is None:
      exit(1)
  
  def start(self):
    self.motionProxy.wakeUp()
    self._look_at(1)
    self._look_at(0)
    self._look_at(-1)

  def _look_at(self, direction):
    service = rospy.get_param('look_at_service')
    rospy.wait_for_service(service)
    try:
      look_at = rospy.ServiceProxy(service, LookAt)
      return look_at(direction).ready
    except rospy.ServiceException as e:
      print("Service call failed: %s" % e)
      return False


def main():
  MasterNode().start()


if __name__ == "__main__":
  main()
