import os
import cv2
import sys
sys.path.append("/home/aneezahm001/Desktop/ros/catkin_ws/src/beginner_tutorials/scripts")

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from hybrid_slam.msg import stereo_img
import numpy as np
import rospy
from dataloader import LoadImages
import yaml


def StereoImgNode():
    pub = rospy.Publisher('sensor_stereo', stereo_img, queue_size=1)
    rospy.sleep(10)
    rospy.init_node('data_publisher', anonymous=True)
    bridge = CvBridge()

    #dataset_left = LoadImages("/home/aneezahm001/Desktop/hybrid_slam/src/hybrid_slam/data/KITTI/dataset/sequences/00/image_0/")
    #dataset_right = LoadImages("/home/aneezahm001/Desktop/hybrid_slam/src/hybrid_slam/data/KITTI/dataset/sequences/00/image_1/")

    # Open the YAML file for reading
    with open("/home/aneezahm001/Desktop/hybrid_slam/src/hybrid_slam/configs/configs_DeepPerception.yaml", 'r') as file:
        data = yaml.safe_load(file)
    
    dataset_left = LoadImages(data["left_imgs_path"])
    dataset_right = LoadImages(data["right_imgs_path"])
    sleep_time = float(data["publish_rate"])

    rate = rospy.Rate(10) # 10hz
    
    sequence_len = dataset_left.getSize()
    assert dataset_left.getSize() == dataset_right.getSize()
    frame_idx = 0

    while not rospy.is_shutdown() and frame_idx < sequence_len:
        path_l, im_l, im0s_l, vid_cap_l, s_l = dataset_left.getFrame(frame_idx)
        path_r, im_r, im0s_r, vid_cap_r, s_r = dataset_right.getFrame(frame_idx)

        timestamp = rospy.Time.now()

        env_name = os.environ.get('CONDA_DEFAULT_ENV')

        img_msg = stereo_img()
        img_msg.path = path_l
        img_msg.left_image = bridge.cv2_to_imgmsg(im0s_l, "bgr8")
        img_msg.right_image = bridge.cv2_to_imgmsg(im0s_r, "bgr8")
        img_msg.left_image.header.stamp = timestamp
        img_msg.right_image.header.stamp = timestamp
        img_msg.left_image.header.frame_id = str(frame_idx)
        img_msg.right_image.header.frame_id = str(frame_idx)
        img_msg.header.stamp = timestamp
        
        pub.publish(img_msg)
        frame_idx += 1

        rospy.sleep(sleep_time)
        #rospy.sleep(0.2)
        #rate.sleep()

if __name__ == '__main__':
    try:
        StereoImgNode()
    except rospy.ROSInterruptException:
        pass
