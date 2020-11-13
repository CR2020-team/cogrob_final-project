#!/usr/bin/python
import numpy as np
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_msgs.srv import LookAt, LookAtResponse


class HeadNode(NaoqiNode):

  CAMERA_FOV = np.deg2rad(55.2)

  __slots__ = 'motionProxy'

  def __init__(self):
    NaoqiNode.__init__(self, 'master_node')
    self.connectNaoQi()

  def connectNaoQi(self):
    rospy.loginfo("HeadNode connecting to NaoQi at %s:%d", self.pip, self.pport)
    self.motionProxy = self.get_proxy("ALMotion")
    if self.motionProxy is None:
      exit(1)
  
  def start(self):
    rospy.Service(rospy.get_param('look_at_service'), LookAt, self.handle_look_at)

  def handle_look_at(self, req):
    self.motionProxy.setStiffnesses("Head", 1.0)
    names  = ["HeadPitch", "HeadYaw"]
    angles  = [0, self.CAMERA_FOV * req.direction]
    fractionMaxSpeed  = 0.5
    self.motionProxy.setAngles(names, angles, fractionMaxSpeed)
    while True:
      currentAngles = self.motionProxy.getAngles(names, True)
      if np.all(np.array(angles) - np.array(currentAngles) < 1e-3)):
        break
    return LookAtResponse(True)


def main():
  HeadNode().start()


if __name__ == "__main__":
  main()
