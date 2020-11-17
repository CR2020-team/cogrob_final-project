#!/usr/bin/python
from collections import Counter
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_msgs.msg import DetectionArrayWithDirection


class SpeakerNode(NaoqiNode):

  __slots__ = 'textToSpeechProxy', 'text', '_direction_to_text', '_counter'

  def __init__(self):
    NaoqiNode.__init__(self, 'speaker_node')
    self.connectNaoQi()
    self._direction_to_text = {
      -1: "on the left.",
      0: "in front of me, ",
      1: "on the right, ",
    }
    self.text = "I can see: "
    self._counter = 0

  def connectNaoQi(self):
    self.pip = rospy.get_param('pip')
    self.pport = rospy.get_param('pport')    
    rospy.loginfo("SpeakerNode connecting to NaoQi at %s:%d", self.pip, self.pport)
    self.textToSpeechProxy = self.get_proxy("ALTextToSpeech")
    if self.textToSpeechProxy is None:
      exit(1)
    self.textToSpeechProxy.setLanguage("English")
  
  def start(self):
    rospy.Subscriber(rospy.get_param('object_list_topic'), DetectionArrayWithDirection, self.rcv_detections_cb)

  def rcv_detections_cb(self, msg):
    self._counter += 1
    d = Counter(detection.clabel for detection in msg.detections)
    items = [self._item_to_text(key, value) for key, value in d.items()]
    self.text += ' and '.join([', '.join(items[:-1]), items[-1]]) + " " + self._direction_to_text[msg.direction]
    if self._counter == len(self._direction_to_text):
      self.textToSpeechProxy.say(self.text)
      
      rospy.loginfo(self.text)  # FIXME

      self.text = "I can see: "
      self._counter = 0

  def _item_to_text(self, key, value):
    return "a {}".format(key) if value == 1 else "{} {}s".format(value, key)


def main():
  SpeakerNode().start()
  try:
    rospy.spin()
  except (KeyboardInterrupt, rospy.exceptions) as e:
    rospy.loginfo("shutdown: %s" % e)

if __name__ == "__main__":
    main()
