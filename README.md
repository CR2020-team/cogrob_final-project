# cogrob_midterm-project

Midterm project for Cognitive Robotics 2020/2021 - Group 26

## Assignment

[assignment.pdf](assignment.pdf)

## Team Members

* [Giovanni Ammendola](https://github.com/giorge1)
* [Edoardo Maffucci](https://github.com/emaff)
* [Vincenzo Petrone](https://github.com/v8p1197)
* [Salvatore Scala](https://github.com/knowsx2)

## Architecture

The project is organized into 5 ROS nodes:

* [camera_node](pepper_pkg/src/camera_node/main.py)
  * Makes Pepper take pictures
* [detector_node](pepper_pkg/src/detector_node/main.py)
  * Performs object detections on pictures taken 
* [head_node](pepper_pkg/src/head_node/main.py)
  * Makes Pepper move the head left and right
* [master_node](pepper_pkg/src/master_node/main.py)
  * Coordinates the camera node and the head node: the picture is taken only when the head is in the correct position
* [speaker_node](pepper_pkg/src/speaker_node/main.py)
  * Makes Pepper speak, saying what she sees around 

## Documentation

### Code

For futher information about the implementation, please refer to the in-code documentation. Every node is fully documentated in the related `main.py` file.

### Messages

The following messages are defined in the [msg](pepper_msgs/msg) folder.

#### DetectionArrayWithDirection

This message is used by two nodes:

* [detector_node](pepper_pkg/src/detector_node/main.py): for each image it receives, it publishes a message with the following fields:
  * **detections:** the classes of the objects seen in the image, along with the confidence scores
  * **direction:** the direction at which the image is taken
* [speaker_node](pepper_pkg/src/speaker_node/main.py): the node is a listener for this message. For each message received, it creates a string that specifies both the objects and the directions.
  * For example:
    * > I can see a bottle on the right, a PC in front of me, 2 TVs on the let.

#### DetectionWithScore

This message is used by only one node:

* [detector_node](pepper_pkg/src/detector_node/main.py):
  * For each object it detects, it creates a message with the following fields:
    * **clabel:** the class label of the object
    * **score:** the confidence score for the detectione
  * This message is then added to the *detections* component of the [DetectionArrayWithDirection](README.md#DetectionArrayWithDirection) message

#### ImageWithDirection

### Services

The following services are defined in the [srv](pepper_msgs/srv) folder.

#### LookAt

#### TakePicture

## Usage

One can simply run the software with

``` bash
roslaunch pepper_pkg pepper.launch
```
