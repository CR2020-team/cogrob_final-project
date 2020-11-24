#!/usr/bin/python
"""
This node represents moves the robot head in the desidered position.
"""

import numpy as np
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_msgs.srv import LookAt, LookAtResponse


"""Class used as an abstraction of the Node"""
class HeadNode(NaoqiNode):

  CAMERA_FOV = np.deg2rad(55.2)

  __slots__ = 'motionProxy'

  def __init__(self):
    """
    Constructor. Creates the node and connects it to the NaoQi interface.
    """
    NaoqiNode.__init__(self, 'master_node')
    self.connectNaoQi()

  def connectNaoQi(self):
    """
    Connects the node to the NaoQi interface. The parameters pip and pport are stored in the parameter server.
    The Proxy used is ALMotion, in order to make the robot head go to the desidered position
    when the service is called.
    """
    self.pip = rospy.get_param('pip')
    self.pport = rospy.get_param('pport')    
    rospy.loginfo("HeadNode connecting to NaoQi at %s:%d", self.pip, self.pport)
    self.motionProxy = self.get_proxy("ALMotion")
    if self.motionProxy is None:
      exit(1)
    rospy.loginfo("ALMotion successful!")
  
  def start(self):
    """
    Actual execution of the node.
    Creates the service server.
    """
    rospy.Service(rospy.get_param('look_at_service'), LookAt, self.handle_look_at)

  def handle_look_at(self, req):
    """
    The handler of the service request.
    The request is the desidered head position (right, front, left) represented as integers (-1, 0, 1).
    The response is True when the position is reached (with an error less than 0.1 rad).
    """
    self.motionProxy.setStiffnesses("Head", 1.0)
    names = ["HeadPitch", "HeadYaw"]
    angles = [np.deg2rad(15), self.CAMERA_FOV * req.direction]
    fractionMaxSpeed = 0.25
    self.motionProxy.setAngles(names, angles, fractionMaxSpeed)
    while True:
      currentAngles = self.motionProxy.getAngles(names, True)
      if np.all(np.array(angles) - np.array(currentAngles) < 1e-1):
        break
    return LookAtResponse(True)


def main():
  """Creates and executes the node."""
  HeadNode().start()
  try:
    rospy.spin()
  except (KeyboardInterrupt, rospy.exceptions) as e:
    rospy.loginfo("shutdown: %s" % e)


if __name__ == "__main__":
  main()
