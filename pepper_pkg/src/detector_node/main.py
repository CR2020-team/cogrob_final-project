#!/usr/bin/python3
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'src'))
import glob
import cv2
import numpy as np
from PIL import Image
import rospy
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
from sr_detector_msgs.msg import Detection2DArrayWithImageName
from classmap import category_map as classmap  # https://gist.github.com/xhlulu/f7735970704b97fd0b72203628c1cc77
from detector import Detector


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
    self._pub = rospy.Publisher(rospy.get_param('object_list_topic'), Detection2DArrayWithImageName, queue_size=0)
    DET_PATH = os.path.join(os.path.dirname(__file__), 'models', detector_model_name, 'saved_model')
    rospy.loginfo("Loading model...")
    self.detector = Detector(DET_PATH)
    rospy.loginfo("Model loaded!")
    self.image_paths = glob.glob(os.path.join(os.path.dirname(__file__), '..', '..', 'images', 'tests', '*.jpg'))
    rospy.loginfo(f"{len(self.image_paths)} images found")

  def __call__(self, image_path: str, verbose=False):
    image = load_image_into_numpy_array(image_path)
    image_name = image_path[image_path.rfind('/') + 1 : image_path.rfind('.')]
    if verbose:
      rospy.loginfo(f"Running inference on {image_name}")
    detections = self.detector(image, self.SCORE_TH)
    if verbose:
      rospy.loginfo(f"{detections['num_detections']} objects found in {image_name}")
    message = Detection2DArrayWithImageName()
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
        message.image = CvBridge().cv2_to_imgmsg(image)
        message.image_name.data = image_name
    self._pub.publish(message)
    if verbose:
      rospy.loginfo(f"Detections published for image {image_name}")

  def start(self, verbose=False):
    for image_path in self.image_paths:
      self(image_path, verbose=verbose)
    rospy.loginfo(f"All {len(self.image_paths)} images with detections published. Exiting ...")


def main():
  try:
    DetectorNode().start(verbose=True)
  except KeyboardInterrupt | rospy.exceptions:
    rospy.loginfo(f"Shutting down Detector Node")
    

if __name__ == "__main__":
  main()
