#!/usr/bin/python
"""
This node is the one that makes the robot speak. It will say what the received detections contain.
"""

from collections import Counter
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_msgs.msg import DetectionArrayWithDirection


"""Class used as an abstraction of the Node"""
class SpeakerNode(NaoqiNode):

  __slots__ = 'animatedSpeechProxy', '_text_components', '_direction_to_text', '_counter'

  def __init__(self):
    """
    Constructor. Creates the node and connects it to the NaoQi interface.
    It also initializes internal variables used to create and coordinate what the robot has to say.
    """
    NaoqiNode.__init__(self, 'speaker_node')
    self.connectNaoQi()
    self._direction_to_text = {
      1: "on the left.",
      0: "in front of me, ",
      -1: "on the right, ",
    }
    self._text_components = {key: None for key in self._direction_to_text.keys()}
    self._counter = 0

  def connectNaoQi(self):
    """
    Connects the node to the NaoQi interface. The parameters pip and pport are stored in the parameter server.
    The Proxy used is ALAnimatedSpeech, in order to make the robot speak when the sentence is ready and
    make the robot move while speaking.
    """
    self.pip = rospy.get_param('pip')
    self.pport = rospy.get_param('pport')    
    rospy.loginfo("SpeakerNode connecting to NaoQi at %s:%d", self.pip, self.pport)
    self.animatedSpeechProxy = self.get_proxy("ALAnimatedSpeech")
    if self.animatedSpeechProxy is None:
      exit(1)
    self.animatedSpeechProxy.setBodyLanguageModeFromStr("contextual")
    # self.animatedSpeechProxy.setParameter("speed", 75)
    rospy.loginfo("ALAnimatedSpeech successful!")
  
  def start(self):
    """
    Actual execution of the node.
    The only operation performed when the node starts is the subscription to the DetectionArrayWithDirection topic and the linking of the callback.
    """
    rospy.Subscriber(rospy.get_param('object_list_topic'), DetectionArrayWithDirection, self.rcv_detections_cb)

  def rcv_detections_cb(self, msg):
    """
    Actual execution of the node. Upon receiving a new message on the topic, it stores a part of the sentence to say. 
    The TTS will be performed when the three directions are all into the sentence to say. In addition, if no object is detected on 
    one side, the robot will say that it sees nothing.
    """
    direction = msg.direction
    self._counter += 1
    d = Counter(detection.clabel for detection in msg.detections)
    items = [self._item_to_text(key, value) for key, value in d.items()]
    if len(items) > 0:
        self._text_components[direction] = ' and '.join([', '.join(items[:-1]), items[-1]]) + " " + self._direction_to_text[direction]
    else:
        self._text_components[direction] = "nothing " + self._direction_to_text[msg.direction]
    if self._counter == len(self._direction_to_text):
      text = "I can see: " + self._text_components[-1] + self._text_components[0] + self._text_components[1]
      
      rospy.loginfo(text)  # FIXME
      self.animatedSpeechProxy.say(text)
      
      self._counter = 0

  def _item_to_text(self, key, value):
    """
    Utility function that return the objects with the corresponding article or number.
    """
    return "a {}".format(key) if value == 1 else "{} {}s".format(value, key)


def main():
  """Creates and executes the node."""
  SpeakerNode().start()
  try:
    rospy.spin()
  except (KeyboardInterrupt, rospy.exceptions) as e:
    rospy.loginfo("shutdown: %s" % e)

if __name__ == "__main__":
    main()
