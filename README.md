# object-detection-ros-cpp
This repository contains ROS-implementation of an object detector in c++ using [OpenCV's dnn module](https://learnopencv.com/deep-learning-with-opencvs-dnn-module-a-definitive-guide/#what-different-models-does-it-support). This is the Capstone project of Udacity's C++ Nanodegree.

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


## Rubric Points

|DONE | CRITERIA | MEETS SPECIFICATIONS| WHERE |
|-- | -- | --| -- |
| * | A README with instructions is included with the project |The README is included with the project and has instructions for building/running the project. If any additional libraries are needed to run the project, these are indicated with cross-platform installation instructions. You can submit your writeup as markdown or pdf.| |
| * | The README indicates which project is chosen. | The README describes the project you have built. The README also indicates the file and class structure, along with the expected behavior or output of the program. | |
| * | The README includes information about each rubric point addressed. | The README indicates which rubric points are addressed. The README also indicates where in the code (i.e. files and line numbers) that the rubric points are addressed. | |

__Compiling and Testing (All Rubric Points REQUIRED)__

|DONE | CRITERIA | MEETS SPECIFICATIONS| WHERE |
|-- | -- | --| -- |
| * | The submission must compile and run. | The project code must compile and run without errors. We strongly recommend using cmake and make, as provided in the starter repos. If you choose another build system, the code must compile on any reviewer platform. |

__Loops, Functions, I/O__

|DONE | CRITERIA | MEETS SPECIFICATIONS| WHERE |
|-- | -- | --| -- |
| * | The project demonstrates an understanding of C++ functions and control structures.| A variety of control structures are used in the project. The project code is clearly organized into functions.| object_detection_node.cpp and object_detector.cpp |
| * | The project reads data from a file and process the data, or the program writes data to a file. | The project reads data from an external file or writes data to a file as part of the necessary operation of the program.| Reads config and model files in object_detector.cpp  |
| * | The project accepts user input and processes the input.|The project accepts input from a user as part of the necessary operation of the program.|  a config file is read and parsed |

__Object Oriented Programming__

|DONE | CRITERIA | MEETS SPECIFICATIONS| WHERE |
|-- | -- | --| -- |
| * | The project uses Object Oriented Programming techniques. | The project code is organized into classes with class attributes to hold the data, and class methods to perform tasks. | All *.cpp and *.h files |
| * | Classes use appropriate access specifiers for class members. | All class data members are explicitly specified as public, protected, or private.| All *.cpp and *.h files |
| * | Class constructors utilize member initialization lists. | All class members that are set to argument values are initialized through member initialization lists.| All *.cpp and *.h files |
| * | Classes abstract implementation details from their interfaces. | All class member functions document their effects, either through function names, comments, or formal documentation. Member functions do not change program state in undocumented ways.| All *.cpp and *.h files |
| * | Classes encapsulate behavior. | Appropriate data and functions are grouped into classes. Member data that is subject to an invariant is hidden from the user. State is accessed via member functions.| All *.cpp and *.h files |
| * | Classes follow an appropriate inheritance hierarchy. | Inheritance hierarchies are logical. Composition is used instead of inheritance when appropriate. Abstract classes are composed of pure virtual functions. Override functions are specified.| class ObjectDetectionNode inherits from ObjectDetector |
|  | Overloaded functions allow the same function to operate on different parameters. |  |
|  | Derived class functions override virtual base class functions. |One member function in an inherited class overrides a virtual base class member function.| |
| * | Templates generalize functions in the project. | One function is declared with a template that allows it to accept a generic parameter.| Done in the utils.h file |

__Memory Management__

|DONE | CRITERIA | MEETS SPECIFICATIONS| WHERE |
|-- | -- | --| -- |
| * | The project makes use of references in function declarations. | At least two variables are defined as references, or two functions use pass-by-reference in the project code.|Extensively done so in multiple functions in object_detector.h and object_detector.cpp |
| | The project uses destructors appropriately. | At least one class that uses unmanaged dynamically allocated memory, along with any class that otherwise needs to modify state upon the termination of an object, uses a destructor. | 
|  | The project uses scope / Resource Acquisition Is Initialization (RAII) where appropriate. | The project follows the Resource Acquisition Is Initialization pattern where appropriate, by allocating objects at compile-time, initializing objects when they are declared, and utilizing scope to ensure their automatic destruction.| |
| | The project follows the Rule of 5. | For all classes, if any one of the copy constructor, copy assignment operator, move constructor, move assignment operator, and destructor are defined, then all of these functions are defined.|  |
| | The project uses move semantics to move data, instead of copying it, where possible. | For classes with move constructors, the project returns objects of that class by value, and relies on the move constructor, instead of copying the object. |  |
| * | The project uses smart pointers instead of raw pointers. | The project uses at least one smart pointer: unique_ptr, shared_ptr, or weak_ptr. The project does not use raw pointers.| used in object_detection_node.cpp |

__Concurrency__

|DONE | CRITERIA | MEETS SPECIFICATIONS| WHERE |
|-- | -- | --| -- |
| | The project uses multithreading. | The project uses multiple threads in the execution.| |
|  | A promise and future is used in the project. | A promise and future is used to pass data from a worker thread to a parent thread in the project code.| |
| | A mutex or lock is used in the project. | A mutex or lock (e.g. std::lock_guard or `std::unique_lock) is used to protect data that is shared across multiple threads in the project code.| |
|  | A condition variable is used in the project. | A std::condition_variable is used in the project code to synchronize thread execution.| |
