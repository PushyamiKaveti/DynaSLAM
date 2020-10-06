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
#include<chrono>
#include<iomanip>
#include <unistd.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"


#include "../../../include/Geometry.h"
#include "../../../include/MaskNet.h"


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(DynaSLAM::SegmentDynObject* mask_net, string d):MaskNet(mask_net), dir(d){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void GrabImages(const sensor_msgs::ImageConstPtr& msg_1, const sensor_msgs::ImageConstPtr& msg_2,\
                     const sensor_msgs::ImageConstPtr& msg_3,const sensor_msgs::ImageConstPtr& msg_4, const sensor_msgs::ImageConstPtr& msg_5);
    DynaSLAM::SegmentDynObject *MaskNet;
    string dir;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 2)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }

    DynaSLAM::SegmentDynObject *MaskNet;
    cout << "Loading Mask R-CNN. This could take a while..." << endl;
    MaskNet = new DynaSLAM::SegmentDynObject();
    cout << "Mask R-CNN loaded!" << endl;
    string dir = string(argv[1]);
    ImageGrabber igb(MaskNet, dir);

    ros::NodeHandle nodeHandler;
    //ros::Subscriber sub = nodeHandler.subscribe("/camera_array/cam1/image_raw", 2, &ImageGrabber::GrabImage,&igb);


    message_filters::Subscriber<sensor_msgs::Image> sub_img_1(nodeHandler, "/camera_array/cam0/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_img_2(nodeHandler, "/camera_array/cam1/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_img_3(nodeHandler, "/camera_array/cam2/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_img_4(nodeHandler,  "/camera_array/cam3/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_img_5(nodeHandler, "/camera_array/cam4/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img_1, sub_img_2, sub_img_3, sub_img_4, sub_img_5);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabImages,&igb, _1, _2 , _3, _4,_5)); //, _5));

    ros::spin();

    ros::shutdown();

    return 0;
}
void ImageGrabber::GrabImages(const sensor_msgs::ImageConstPtr& msg_1, const sensor_msgs::ImageConstPtr& msg_2,\
                     const sensor_msgs::ImageConstPtr& msg_3,const sensor_msgs::ImageConstPtr& msg_4, const sensor_msgs::ImageConstPtr& msg_5) {
    //ROS_INFO_STREAM("HERE");
    vector<cv::Mat> imgs ;
    try{
        imgs.push_back(cv_bridge::toCvShare(msg_1, "mono8")->image);
        imgs.push_back(cv_bridge::toCvShare(msg_2, "mono8")->image);
        imgs.push_back(cv_bridge::toCvShare(msg_3, "mono8")->image);
        imgs.push_back(cv_bridge::toCvShare(msg_4, "mono8")->image);
        imgs.push_back(cv_bridge::toCvShare(msg_5, "mono8")->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //if (imgs[0].empty() || imgs[1].empty() || imgs[2].empty() || imgs[3].empty() ||imgs[4].empty() )
    //    return;
    string name = to_string(msg_2->header.stamp.sec)+ "." + to_string(msg_2->header.stamp.nsec) + ".bmp";
    for(uint i =0; i < imgs.size(); i++){
        cv::Mat maskRCNN;
        cv::Mat imRGB = imgs[i].clone();
        maskRCNN = MaskNet->GetSegmentation(imRGB,dir+"/cam"+to_string(i),name);
    }


}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat maskRCNN;
    cv::Mat imRGB = cv_ptr->image.clone();
    string name = to_string(msg->header.stamp.sec)+ "." + to_string(msg->header.stamp.nsec) + ".bmp";
    maskRCNN = MaskNet->GetSegmentation(imRGB,dir,name);

}


