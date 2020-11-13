#!/usr/bin/python3
import cv2
import numpy as np
from PIL import Image
import rospy
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
from classmap import category_map as classmap  # https://gist.github.com/xhlulu/f7735970704b97fd0b72203628c1cc77
from detector import Detector
from pepper_msgs.msg import ImageWithDirection, Detection2DArrayWithDirection


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


class DetectorNode:

  SCORE_TH = 0.45

  __slots__ = 'detector', '_pub', 'image_paths'

  def __init__(self, detector_model_name='efficientdet_d7_coco17_tpu-32'):
    rospy.init_node('detector_node')
    DET_PATH = os.path.join(os.path.dirname(__file__), 'models', detector_model_name, 'saved_model')
    rospy.loginfo("Loading model...")
    self.detector = Detector(DET_PATH)
    rospy.loginfo("Model loaded!")

  def __call__(self, image, direction, verbose=False):
    detections = self.detector(image, self.SCORE_TH)
    if verbose:
      rospy.loginfo(f"{detections['num_detections']} objects found at {direction}")
    message = Detection2DArrayWithDirection()
    for clabel, score, box in zip(detections['detection_classes'], detections['detection_scores'], detections['detection_boxes']):
      if clabel in classmap:
        d = Detection2D()
        d.bbox.size_x = box[3] - box[1]
        d.bbox.size_y = box[2] - box[0]
        d.bbox.center.x = box[1] + d.bbox.size_x / 2
        d.bbox.center.y = box[0] + d.bbox.size_y / 2
        o = ObjectHypothesisWithPose()
        o.score = score
        o.id = clabel
        d.results.append(o)
        message.detections.detections.append(d)
    message.direction = direction
    self._pub.publish(message)

  def start(self, verbose=False):
    rospy.Subscriber(rospy.get_param('image_topic'), ImageWithDirection, self.rcv_image_cb)
    self._pub = rospy.Publisher(rospy.get_param('object_list_topic'), Detection2DArrayWithDirection, queue_size=0)

    for image_path in self.image_paths:
      self(image_path, verbose=verbose)
    rospy.loginfo(f"All {len(self.image_paths)} images with detections published. Exiting ...")

  def rcv_image_cv(self, msg):
    self(CvBridge().imgmsg_to_cv2(msg.image), msg.direction)


def main():
  try:
    DetectorNode().start(verbose=True)
  except KeyboardInterrupt | rospy.exceptions:
    rospy.loginfo(f"Shutting down Detector Node")
    

if __name__ == "__main__":
  main()
