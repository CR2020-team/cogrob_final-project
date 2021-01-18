# cogrob_final-project

Final project for Cognitive Robotics 2020/2021 - Group 26

## Assignment

[assignment.pdf](doc/assignment.pdf)

## Team Members

* [Giovanni Ammendola](https://github.com/giorge1)
* [Edoardo Maffucci](https://github.com/emaff)
* [Vincenzo Petrone](https://github.com/v8p1197)
* [Salvatore Scala](https://github.com/knowsx2)

## Architecture

The project is organized into 4 ROS nodes:

* [camera_node](pepper_pkg/src/camera_node/camera_node)
  * Makes Pepper take pictures
* [detector_node](pepper_pkg/src/detector_node/detector_node)
  * Performs object detections on pictures taken 
* [head_node](pepper_pkg/src/head_node/head_node)
  * Makes Pepper move the head left and right
* [speaker_node](pepper_pkg/src/speaker_node/speaker_node)
  * Makes Pepper speak, saying what she sees around 

## Documentation

### Code

For futher information about the implementation, please refer to the in-code documentation. Every node is fully
documentated in the related file.

### Messages

The following messages are defined in the [msg](pepper_msgs/msg) folder.

#### DetectionWithScore

This message is used by only one node:

* [detector_node](pepper_pkg/src/detector_node/detector_node):
  * For each object it detects, it creates a message with the following fields:
    * `clabel`: the class label of the object
    * `score`: the confidence score for the detection
  * This message is then added to the `detections` component of the [SayDetections](README.md#SayDetections)
    service request
    
### Services

The following services are defined in the [srv](pepper_msgs/srv) folder.

#### TakePicture

This service is served by the [camera node](pepper_pkg/src/camera_node/camera_node) and is requested by the
[head node](pepper_pkg/src/head_node/head_node) in order to take a picture. Since the picture will be forwarded to
the [DetectImage](README.md#DetectImage) service, the requests take as parameter the current position of the head.

* [camera_node](pepper_pkg/src/camera_node/camera_node) upon receiving a request, takes the picture, forwards it
  to the [DetectImage](README.md#DetectImage) service along with the direction and sends a `True` response if and only
  if the operation is successful.
* [head_node](pepper_pkg/src/head_node/head_node) is the only service client. Before taking a picture, the node
  must be sure that the head is in the right position.

#### DetectImage

This service is served by the [detector node](pepper_pkg/src/detector_node/detector_node) and is requested by the
[camera node](pepper_pkg/src/camera_node/camera_node) in order to detect objects in a picture. Since the objects
will be forwarded to the [SayDetections](README.md#SayDetections) service, the requests take as parameter the direction
at which the given image has been taken.

* [detector_node](pepper_pkg/src/detector_node/detector_node) upon receiving a request, gives the picture as input
  to the detector model, creates a [SayDetections](README.md#SayDetections) service request containing the detected
  objects; sends a `True` response if and only if the operation is successful.
* [camera_node](pepper_pkg/src/camera_node/camera_node) is the only service client. Creates a request by passing
  as parameter the image taken and its proper direction

#### SayDetections

This service is served by the [speaker_node](../../pepper_pkg/src/speaker_node/speaker_node) and is requested by the 
[detector_node](../../pepper_pkg/src/detector_node/detector_node) in order to convert a list o detected objects into a
string.

* [speaker_node](../../pepper_pkg/src/speaker_node/speaker_node): upon receiving a request, creates a string depending 
  on the detected objects and the direction at which the objects have been detected.
  * For example:
    > I can see a bottle on the right, a PC in front of me, 2 TVs on the left.
* [detector_node](../../pepper_pkg/src/detector_node/detector_node) is the only service client. Creates a request by 
  passing as parameters:
  * `detections`: the classes of the objects seen in the image, along with the confidence scores
  * `direction`: the direction at which the image has been taken
    
## Usage

To use the software, just run

``` bash
roslaunch pepper_pkg pepper.launch pepper_id:=id
```

Where `id` is either 1 or 2 depending on which pepper robot the software bill be executed. If not specified, `pepper_id` is 1.
