/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include<opencv2/core/core.hpp>

#include<System.h>
#include<GL/glut.h>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "hybrid_slam/array_msg.h"
#include "hybrid_slam/stereo_img.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float32MultiArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define CV_LOAD_IMAGE_UNCHANGED -1

using namespace std;

class Subscriber
{
	public:
    		Subscriber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    		void GrabData(const hybrid_slam::array_msg::ConstPtr& dataMsg, const hybrid_slam::stereo_img::ConstPtr& imageMsg);
    		void GrabData(const hybrid_slam::stereo_img::ConstPtr& imageMsg);

    		ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    //string voc_path = "/home/aneezahm001/Desktop/hybrid_slam/src/hybrid_slam/configs/Vocabulary/ORBvoc.txt";
    //string settings_path = "/home/aneezahm001/Desktop/hybrid_slam/src/hybrid_slam/configs/Settings/KITTI00-02.yaml";
    //string images_path = "/home/aneezahm001/Desktop/slam/data/KITTI/dataset/sequences/03/";

    //vector<vector<float>> recognition_message = read_file();
 
    //cv::FileStorage fs("/home/aneezahm001/Desktop/hybrid_slam/src/hybrid_slam/configs/configs_SLAM.yaml", cv::FileStorage::READ);
    
    cv::FileStorage fs(argv[1], cv::FileStorage::READ);	
    if(!fs.isOpened())
    {
       cerr << "Failed to open configs file." << endl;
       exit(-1);
    }

    string voc_path, settings_path;
    fs["vocabulary_path"] >> voc_path;
    fs["settings_path"] >> settings_path;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(voc_path, settings_path, ORB_SLAM2::System::STEREO, true);

    
    ros::init(argc, argv, "slam");
    ros::NodeHandle n;

    Subscriber igb(&SLAM);

    //ros::Subscriber imageSub = n.subscribe("chatter", 1000, &Subscriber::GrabData, &igb);
    
    message_filters::Subscriber<hybrid_slam::array_msg> dataSub(n, "semantic_percepts", 1000);
    message_filters::Subscriber<hybrid_slam::stereo_img> imageSub(n, "sensor_stereo", 1000);

    typedef message_filters::sync_policies::ApproximateTime<hybrid_slam::array_msg,hybrid_slam::stereo_img> syncPol;
    message_filters::Synchronizer<syncPol> sync(syncPol(10), dataSub, imageSub);
    sync.registerCallback(boost::bind(&Subscriber::GrabData,&igb,_1,_2));
    

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    ros::shutdown();
    
    return 0;
}

void Subscriber::GrabData(const hybrid_slam::array_msg::ConstPtr& dataMsg, const hybrid_slam::stereo_img::ConstPtr& imageMsg){
	auto start = std::chrono::high_resolution_clock::now();

	std::string path = imageMsg->path;

        sensor_msgs::Image left_image = imageMsg->left_image;
        sensor_msgs::Image right_image = imageMsg->right_image;

        cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;

        try{
                cv_ptr_left = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::BGR8);
                cv_ptr_right = cv_bridge::toCvCopy(right_image, sensor_msgs::image_encodings::BGR8);
        } catch(cv_bridge::Exception& e){
                ROS_ERROR("cv bridge exception: %s", e.what());
                return;
        }

        cv::Mat imLeft = cv_ptr_left->image;
        cv::Mat imRight = cv_ptr_right->image;

        std_msgs::Float32MultiArray temp = dataMsg->data;
	std::vector<float> array_data = temp.data;

	vector<vector<float>> semantics;
	int frame_id = 0;
	if(array_data.size() > 0){
		for (int i = 0; i < temp.data.size(); i += 11){
			std::vector<float> chunk(array_data.begin() + i, array_data.begin() + i + 11);
		
			//std::copy(chunk.begin(), chunk.end(), std::ostream_iterator<float>(std::cout, " "));
			//std::cout << std::endl;

			semantics.push_back(chunk);
		}

		frame_id = array_data[0];
	}


	//track frames and estimate time taken
	auto start_slam = std::chrono::high_resolution_clock::now();

	mpSLAM->TrackStereo(frame_id, imLeft,imRight,dataMsg->header.stamp.toSec(), semantics);

	auto end_slam = std::chrono::high_resolution_clock::now();
	auto end = std::chrono::high_resolution_clock::now();

	auto duration_slam = std::chrono::duration_cast<std::chrono::milliseconds>(end_slam - start_slam);
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

	std::cout <<ros::Time::now().toSec() <<": SLAM Node - Elapsed time for Frame " <<frame_id <<" " <<duration_slam.count() << " ms. Elapsed time for iteration " <<duration.count() <<"ms. \n" << std::endl;
}

//for testing slam node without deep perception
void Subscriber::GrabData(const hybrid_slam::stereo_img::ConstPtr& imageMsg){
        std::string path = imageMsg->path;

        sensor_msgs::Image left_image = imageMsg->left_image;
        sensor_msgs::Image right_image = imageMsg->right_image;

        cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right;

        try{
                cv_ptr_left = cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::BGR8);
                cv_ptr_right = cv_bridge::toCvCopy(right_image, sensor_msgs::image_encodings::BGR8);
        } catch(cv_bridge::Exception& e){
                ROS_ERROR("cv bridge exception: %s", e.what());
                return;
        }

        cv::Mat imLeft = cv_ptr_left->image;
        cv::Mat imRight = cv_ptr_right->image;

	vector<vector<float>> semantics;
        mpSLAM->TrackStereo(0, imLeft, imRight, 12.3, semantics);
}

//for testing slam node with precomputed semantic map points
vector<vector<float>> read_file(){
        vector<vector<float>> values;

        ifstream input_file("/home/aneezahm001/Desktop/seq_3.txt");

        if(input_file){
                string line;

                while(getline(input_file, line)){
                        istringstream iss(line);
                        vector<float> line_values;

                        float value;
                        while(iss >> value){
                                line_values.push_back(value);
                        }

                        values.push_back(line_values);
                }

                input_file.close();
        }else{
                cout<<"Could not open file" <<endl;
        }


        return values;

}
