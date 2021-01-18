## Services Documentation

### TakePicture

This service is served by the [camera node](../../pepper_pkg/src/camera_node/camera_node) and is requested by the
[head node](../../pepper_pkg/src/head_node/head_node) in order to take a picture. Since the picture will be forwarded to
the [DetectImage](README.md#DetectImage) service, the requests take as parameter the current position of the head.

* [camera_node](../../pepper_pkg/src/camera_node/camera_node) upon receiving a request, takes the picture, forwards it
  to the [DetectImage](README.md#DetectImage) service along with the direction and sends a `True` response if and only
  if the operation is successful.
* [head_node](../../pepper_pkg/src/head_node/head_node) is the only service client. Before taking a picture, the node
  must be sure that the head is in the right position.

### DetectImage

This service is served by the [detector node](../../pepper_pkg/src/detector_node/detector_node) and is requested by the
[camera node](../../pepper_pkg/src/camera_node/camera_node) in order to detect objects in a picture. Since the objects
will be forwarded to the [SayDetections](README.md#SayDetections) service, the requests take as parameter the direction
at which the given image has been taken.

* [detector_node](../../pepper_pkg/src/detector_node/detector_node) upon receiving a request, gives the picture as input
  to the detector model, creates a [SayDetections](README.md#SayDetections) service request containing the detected
  objects and sends a `True` response if and only if the operation is successful.
* [camera_node](../../pepper_pkg/src/camera_node/camera_node) is the only service client. Creates a request by passing
  as parameter the image taken and its proper direction

### SayDetections

This service is served by the [speaker_node](../../pepper_pkg/src/speaker_node/speaker_node) and is requested by the 
[detector_node](../../pepper_pkg/src/detector_node/detector_node) in order to convert a list o detected objects into a
string.

* [speaker_node](../../pepper_pkg/src/speaker_node/speaker_node): upon receiving a request, creates a string depending 
  on the detected objects and the direction at which the objects have been detected and sends a `True` response if and
  only if the direction is valid
  * For example:
    > I can see a bottle on the right, a PC in front of me, 2 TVs on the left.
* [detector_node](../../pepper_pkg/src/detector_node/detector_node) is the only service client. Creates a request by 
  passing as parameters:
  * `detections`: the classes of the objects seen in the image, along with the confidence scores
  * `direction`: the direction at which the image has been taken