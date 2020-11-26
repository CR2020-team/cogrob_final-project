#!/usr/bin/python3
"""
This node detects objects in images.
"""

import os
import cv2
import numpy as np
from PIL import Image
import rospy
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
from classmap import category_map as classmap  # https://gist.github.com/xhlulu/f7735970704b97fd0b72203628c1cc77
from detector import Detector
from pepper_msgs.msg import ImageWithDirection, DetectionArrayWithDirection, DetectionWithScore
import ros_numpy


def load_image_into_numpy_array(path):
  """Load an image from file into a numpy array.
  Puts image into numpy array to feed into tensorflow graph.
  Note that by convention we put it into a numpy array with shape
  (height, width, channels), where channels=3 for RGB.
  Args:
    path: the file path to the image
  Returns:
    uint8 numpy array with shape (img_height, img_width, 3)
  """
  return np.array(Image.open(path))


"""Class used as an abstraction of the Node"""
class DetectorNode:

  """Detections with a confidence score lower than SCORE_TH will not be considered as actual detections."""
  SCORE_TH = 0.45

  __slots__ = 'detector', '_pub', '_verbose', '_images', '_ready'

  def __init__(self):
    """
    Constructor.
    Creates the node, connects it to the NaoQi interface and subscribes it to the ImageWithDirection topic.
    """
    rospy.init_node('detector_node')
    self._images = {}
    self._ready = False
    rospy.Subscriber(rospy.get_param('image_topic'), ImageWithDirection, self.rcv_image_cb)

  def __call__(self, image, direction, verbose=False):
    """
    Runs inference in image, performing detections on it.
    Publishes a message containing DetectionArrayWithDirection.
    """
    detections = self.detector(image, self.SCORE_TH)
    if verbose:
      rospy.loginfo("{} objects found at {}".format(detections['num_detections'], direction))
    message = DetectionArrayWithDirection()
    for clabel, score in zip(detections['detection_classes'], detections['detection_scores']):
      if clabel in classmap:
        d = DetectionWithScore()
        d.clabel = classmap[clabel]
        d.score = score
        message.detections.append(d)
    message.direction = direction
    self._pub.publish(message)
    rospy.loginfo("Detector published: {}".format(direction))

  def start(self, detector_model_name='efficientdet_d1_coco17_tpu-32', verbose=False):
    """
    Actual execution of the node.
    Creates a pulisher to the DetectionArrayWithDirection topic and loads the detector model.
    """
    self._pub = rospy.Publisher(rospy.get_param('object_list_topic'), DetectionArrayWithDirection, queue_size=0, latch=False)
    self._verbose = verbose
    DET_PATH = os.path.join(os.path.dirname(__file__), 'models', detector_model_name, 'saved_model')
    rospy.loginfo("Loading model...")
    self.detector = Detector(DET_PATH)
    self._ready = True
    rospy.loginfo("Model loaded!")
    for direction, image in self._images.items():
      self(image, direction)

  def rcv_image_cb(self, msg):
    """
    Callback used by the ImageWithDirection subscriber.
    If the model is loaded, performs a detection on the image.
    Otherwise, appends the image in a queue.
    """
    rospy.loginfo("Received image {}".format(msg.direction))
    if self._ready:
      self(ros_numpy.numpify(msg.image), msg.direction)
    else:
      self._images[msg.direction] = ros_numpy.numpify(msg.image)


def main():
  """Creates and executes the node."""
  DetectorNode().start(verbose=True)
  try:
    rospy.spin()
  except (KeyboardInterrupt, rospy.exceptions) as e:
    rospy.loginfo("shutdown: %s" % e)


if __name__ == "__main__":
  main()
