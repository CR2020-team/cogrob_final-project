import tensorflow as tf
assert(int(tf.__version__.split('.')[0]) >= 2)
import numpy as np
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
tf.get_logger().setLevel('ERROR')


class Detector:
    def __init__(self, model_path):
        self.detect_fn = tf.saved_model.load(model_path)

    def __call__(self, img, threshold=0.5):
        img = img[:, :, ::-1]
        input_tensor = tf.convert_to_tensor(img)
        input_tensor = input_tensor[tf.newaxis, ...]
        detections = self.detect_fn(input_tensor)
        num_above_thresh = np.sum(detections['detection_scores'] > threshold)
        detections.pop('num_detections')
        detections = {key: value[0, :num_above_thresh].numpy() for key, value in detections.items()}
        detections['detection_classes'] = detections['detection_classes'].astype(np.int64)
        detections['num_detections'] = num_above_thresh
        return detections
