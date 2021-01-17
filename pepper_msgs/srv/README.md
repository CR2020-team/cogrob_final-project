## Services Documentation

### TakePicture

This service is served by the [camera node](../../pepper_pkg/src/camera_node/camera_node) and is requested by the [head node](../../pepper_pkg/src/head_node/head_node) in order to take a picture. Since the picture will be forwarded to the [DetectImage](README.md#DetectImage) service, the requests take as parameter the current position of the head.

* [camera_node](../../pepper_pkg/src/camera_node/camera_node) upon receiving a request, takes the picture, forwards it to the [DetectImage](README.md#DetectImage) service along with the direction and sends a `True` response if and only if the operation is successful.
* [head_node](../../pepper_pkg/src/head_node/head_node) is the only service client. Before taking a picture, the node must be sure that the head is in the right position.

### DetectImage

This service is served by the [detector node](../../pepper_pkg/src/detector_node/detector_node) and is requested by the [camera node](../../pepper_pkg/src/camera_node/camera_node) in order to detect objects in a picture. Since the picture will be published on a [DetectionArrayWithDirection](../msg/README.md#DetectionArrayWithDirection) message, the requests take as parameter the direction at which the given image has been taken.

* [detector_node](../../pepper_pkg/src/detector_node/detector_node) upon receiving a request, gives the picture as input to the detector model, creates a [DetectionArrayWithDirection](../msg/README.md#DetectionArrayWithDirection) message containing the detected objects and publishes it to the proper topic; sends a `True` response if and only if the operation is successful.
* [camera_node](../../pepper_pkg/src/camera_node/camera_node) is the only service client. Creates a request by passing as parameter the image taken and its proper direction

