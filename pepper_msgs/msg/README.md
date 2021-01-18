## Messages Documentation

### DetectionWithScore

This message is used by only one node:

* [detector_node](../../pepper_pkg/src/detector_node/detector_node):
  * For each object it detects, it creates a message with the following fields:
    * `clabel`: the class label of the object
    * `score`: the confidence score for the detection
  * This message is then added to the `detections` component of the [SayDetections](../srv/README.md#SayDetections)
    service request
    