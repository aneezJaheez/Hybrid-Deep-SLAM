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
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Tested with OpenCV 3.4**.

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

Lastly, the ORB-SLAM vocabulary needs to be specified. A compressed version of this is included in the repository and can be extracted as follows:
```
cd src/hybrid_slam/configs/Vocabulary/
tar -xvf ORBvoc.txt.tar.gz
```

# 4. Dataset

The project can currently be run on stereo image datasets. The framework has been tested on the KITTI Visual Odometry sequences.  

The KITTI VO dataset can be downloaded from the official website. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php and extract it.

# 5. Configuring run-time settings
The project contains several settings that can be configured before run-time. These can be specified in the following .yaml files. 

```
src/hybrid_slam/configs/configs_SLAM.yaml
src/hybrid_slam/configs/configs_DeepPerception.yaml
```

These files need to be configured prior to running the project. In the configs_SLAM.yaml file, the Vocabulary path and settings file path for running ORB-SLAM needs to be specified. 

```
#ORB_SLAM parameters
vocabulary_path: "/home/aneezahm001/Desktop/hybrid_slam/src/hybrid_slam/configs/Vocabulary/ORBvoc.txt"
settings_path: "/home/aneezahm001/Desktop/hybrid_slam/src/hybrid_slam/configs/Settings/KITTI00-02.yaml"
```

The settings file specifies camera calibration settings for SLAM. These files are already included in the repository for the KITTI stereo camera setup and can be found in the following respective folders:
```
src/hybrid_slam/configs/Vocabulary/
src/hybrid_slam/configs/Settings/
```

Example instantiations for these settings are included in the .yaml file.

The configs_DeepPerception.yaml file contains the settings for running the deep perception node as well as the data publisher node. As such, the path to the left and right stereo images needs to be specified. 

```
#Dataset
left_imgs_path: "/home/aneezahm001/Desktop/hybrid_slam/src/hybrid_slam/data/KITTI/dataset/sequences/00/image_0/" #path to left stereo image directory
right_imgs_path: "/home/aneezahm001/Desktop/hybrid_slam/src/hybrid_slam/data/KITTI/dataset/sequences/00/image_1/" #path to right stereo image directory
```

Further, the camera calibration file for the stereo camera setup also needs to be specified. The calibration files for the KITTI dataset are included in the repo (src/hybrid_slam/configs/calibration/). Example instantiations for these parameters for the KITTI VO dataset are included in the .yaml file.

```
cam_calib_file: '/home/aneezahm001/Desktop/slam/data/kitti_mot/calib/calib_cam_to_cam.txt' #Path to stereo camera calibration file
```

In addition, while the project performs stereo depth estimation and triangulation by default, the lidar depth estimates can also be used by modifying the following settings in the configs_DeepPerception.yaml file. The lidar velodyne data can be downloaded from the KITTI website () and its calibration files are included in the aforementioned calibration folder. 

```
use_velodyne: False #use depth information from velodyne lidar sensors. If false, use stereo depth estimation
cam_calib_file: '/home/aneezahm001/Desktop/slam/data/kitti_mot/calib/calib_cam_to_cam.txt' #Path to stereo camera calibration file
velo_calib_file: "/home/aneezahm001/Desktop/slam/data/kitti_mot/calib/calib_velo_to_cam.txt" #Path to velodyne calibration file (optional, only if use_velodyne is set to True)
velo_bin_dir: "/home/aneezahm001/Desktop/slam/data/KITTI/dataset/sequences/03/velodyne/" #path to directory containing velodyne point clouds (optional, only if use_velodyne is set to true)
```

Explanations of additional run-time configurations are included in the .yaml files.

# 6. Running the project

Once the project is built and the run-time configurations are specified in the .yaml files, the project can be run from the base folder by executing the following commands:

```
source devel/setup.bash
roslaunch hybrid_slam final.launch
```

where final.launch is a ros launch file that instantiates roscore and the rosnodes. 

# 7. Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided  KITTI datasets for stereo  cameras. We use the calibration model of OpenCV. Stereo input must be synchronized and rectified. 

## **Reference**
This project is inspired by these works:
* [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2/tree/master)
* [Salt-Pepper Robotics](https://github.com/PaolaArdon/Salt-Pepper) for the Hybrid ROS Architecture
* [YOLO](https://github.com/ultralytics/ultralytics) for the object detection and segmentation pipeline

---
## **Special thanks**
I want to thank my supervisor, Assoc. Professor Lam Siew Kei, for his constant supervision, invaluable assistance and support.
