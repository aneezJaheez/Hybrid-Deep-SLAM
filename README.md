# Hybrid SLAM and Multi-Task Deep Perception
This is the Final Year Project of Aneez Ahmed Jaheezuddin, CS Batch 2019, for the Bachelor of Computer Science in NTU SCSE

# 1. Prerequisites
The project is tested on Ubuntu **20.04**. A powerful computer (e.g. i7) equipped with a GPU will ensure real-time performance.

## C++14 or C++0x Compiler
The project has been tested using C++14.

## Python 3.7
The project has been tested using Python 3.7.

## ROS Noetic
The project has been tested on ROS Noetic. Ensure that the ROS session is sourced using the appropriate python environment to ensure that the right dependencies are being used during run-time. 

# 2. Installing Dependenices

Clone the repository:
```
git clone https://github.com/aneezJaheez/semantic-slam.git
```

The repository requires the Python and C++ pre-requisites to be installed separately. 

## Python 

As stated, the project has been tested using Python 3.7. After cloning the repository, the Python dependencies can be installed using the following commands:

```
cd hybrid_slam
pip install -r requirements.txt
```

The python module uses rospy and will hence require ROS to be installed prior. 

## C++

The C++ dependencies are highlighted below:

### Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

### OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

### Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

### DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

# 3. Building the Project

Once all the prerequisites have been downloaded, the project can be built. This can be done in a few steps. 

First, install the Thirdparty dependecies (Included in the Thirdparty folder). From the project base directory, execute the following commands. 

To build DBoW2:
```
cd src/hybrid_slam/src/ORB_SLAM2/Thirdparty/DBoW2/
mkdir build && cd build
cmake ..
make
```

To build g20:
```
cd src/hybrid_slam/src/ORB_SLAM2/Thirdparty/g2o/
mkdir build && cd build
cmake ..
make
```

Once the thirdparty dependencies have been build, the entire ROS project can be built. From the project base directory, execute the following command:
```
catkin_make
```



