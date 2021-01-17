## Messages Documentation

#### DetectionArrayWithDirection

This message is used by two nodes:

* [detector_node](../../pepper_pkg/src/detector_node/detector_node): for each image it receives, it publishes a message with the following fields:
  * `detections`: the classes of the objects seen in the image, along with the confidence scores
  * `direction`: the direction at which the image has been taken
* [speaker_node](../../pepper_pkg/src/speaker_node/speaker_node): the node is a listener for this message. For each message received, it creates a string that specifies both the objects and the directions.
  * For example:
    > I can see a bottle on the right, a PC in front of me, 2 TVs on the left.

#### DetectionWithScore

This message is used by only one node:

* [detector_node](../../pepper_pkg/src/detector_node/detector_node):
  * For each object it detects, it creates a message with the following fields:
    * `clabel`: the class label of the object
    * `score`: the confidence score for the detection
  * This message is then added to the `detections` component of the [DetectionArrayWithDirection](README.md#DetectionArrayWithDirection) message
    