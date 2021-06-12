# object-detection-ros-cpp
This repository contains ROS-implementation of an object detector in c++ using OpenCV's dnn module. This is the Capstone project of Udacity's C++ Nanodegree.

<img src="imgs/objectDetection.gif"/>

# Installation
1. Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/).
2. (Optional) Follow [Post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) in order to run without root privileges.
3. Download repository
    ```bash
    git clone https://github.com/jdgalviss/object-detection-ros-cpp.git
    ```
4. Build docker countainer
    ```bash
    cd object-detection-ros-cpp
    docker build . -t object_detection
    ```
5. (Optional) Allow the container to display images
    ```bash
    xhost +local:root
    ```
6. Make sure you have a camera device connected and that it is visible:
    ```bash
    ls /dev/video0 
    ```
    This should print: **/dev/video0** 

7. Run the container ()

    With image display:

    ```bash
    docker run --net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --env QT_X11_NO_MITSHM=1 -v `pwd`/object_detection:/home/developer/object_detection -p 8888:8888 --device /dev/video0 -it --rm object_detection
    ```

    Without image display:

    ```bash
    docker run --net=host -v `pwd`/object_detection:/home/developer/object_detection -p 8888:8888 --device /dev/video0 -it --rm object_detection
    ```
8. Inside the container, build the project
    ```bash
    source /opt/ros/noetic/setup.bash
    catkin_make
    source devel/setup.bash
    ```

9. Run the example launch file
    ```bash
    roslaunch object_detection example.launch
    ```
10. Aditionally, there is a launch file ([example_video.launch](object_detection/catkin_ws/src/objec_detection/launch/example_video.launch)) that takes a video as input instead of camera images.
## Using docker (recommended)

## On host (tested on UBuntu 20.04 and ROS noetic)

1. Install [ROS noetic](https://docs.ros.org/en/noetic/Installation/Linux-Install-Debians.html)  
2. Install required packages
    ```bash
    apt-get install ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-cv-camera
    ```
3. Download repository
    ```bash
    git clone https://github.com/jdgalviss/object-detection-ros-cpp.git
    ```
4. Go to the catkin workspace and build it.
    ```bash
    cd object-detection-ros-cpp/object_detection/catkin_ws
    source /opt/ros/noetic/setup.bash
    catkin_make
    source devel/setup.bash
    ```
5. Make sure you have a camera device connected and that it is visible:
    ```bash
    ls /dev/video0 
    ```
    This should print: **/dev/video0** 

6. Modify the *class_filename, model_filename and config_file* parameters in the config file accordingly.

7. Run the example launch file
    ```bash
    roslaunch object_detection example.launch
    ```

## Configuration

You can define your own configuration file (Check for example the [params_mobilenet_ssd.yaml](object_detection/catkin_ws/src/object_detection/config/paras.yaml) file).

```bash
model:
  class_filename: "/home/developer/object_detection/catkin_ws/src/object_detection/model/object_detection_classes_coco.txt"
  model_filename: "/home/developer/object_detection/catkin_ws/src/object_detection/model/frozen_inference_graph.pb"
  config_file: "/home/developer/object_detection/catkin_ws/src/object_detection/model/ssd_mobilenet_v2_coco_2018_03_29.pbtxt.txt"
  score_threshold: 0.4
display:
  imshow: true

# cv-camera params
cv_camera:
  rate: 10
  device_id: 0
  frame_id: "camera"
  image_width: 640
  image_height: 480
```


