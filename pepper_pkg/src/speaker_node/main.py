#!/usr/bin/python
from collections import Counter
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_msgs.msg import Detection2DArrayWithDirection


class SpeakerNode(NaoqiNode):

  __slots__ = 'textToSpeechProxy', 'text'

  def __init__(self):
    NaoqiNode.__init__(self, 'speaker_node')
    self.connectNaoQi()
    self.text = "I can see: "

  def connectNaoQi(self):
    rospy.loginfo("HeadNode connecting to NaoQi at %s:%d", self.pip, self.pport)
    self.textToSpeechProxy = self.get_proxy("ALTextToSpeech")
    if self.textToSpeechProxy is None:
      exit(1)
  
  def start(self):
    rospy.Subscriber(rospy.get_param('object_list_topic'), Detection2DArrayWithDirection, self.rcv_detections_cb)

  def rcv_detections_cb(self, msg):
    d = Counter(detection.id for detection in msg.detections)
    items = d.items()
    for key, value in items[:-1]:
        self.text += self._item_to_text(key, value)
    key, value = items[-1]
    self.text += " and "
    self.text += self._item_to_text(key, value) += " "
    direction_to_text = {
      -1: "on the left.",
      0: "in front of me, ",
      1: "on the right, ",
    }
    self.text += direction_to_text[msg.direction]
    if msg.direction == -1:
      self.textToSpeechProxy.say(self.text)
      self.text = "I can see: "

  def _item_to_text(self, key, value):
    return "a {}".format(key) if value == 1 else "{} {}s".format(value, key) + ", "


def main():
  SpeakerNode().start()
  try:
    rospy.spin()
  except (KeyboardInterrupt, rospy.exceptions) as e:
    rospy.loginfo("shutdown: %s" % e)

if __name__ == "__main__":
    main()
