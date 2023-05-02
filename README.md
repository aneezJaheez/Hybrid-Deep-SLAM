# Hybrid SLAM and Multi-Task Deep Perception
This is the Final Year Project of Aneez Ahmed Jaheezuddin, CS Batch 2019, for the Bachelor of Computer Science in NTU SCSE

# Building the Project

# 2. Prerequisites
The project is tested on Ubuntu **20.04**. A powerful computer (e.g. i7) equipped with a GPU will ensure real-time performance.

## C++11 or C++0x Compiler

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## Python 3.7

# 3. Building the Hybrid SLAM Project

Clone the repository:
```
git clone https://github.com/aneezJaheez/semantic-slam.git
```

The repository requires the Python and C++ pre-requisites to be installed separately. 

## Python 
