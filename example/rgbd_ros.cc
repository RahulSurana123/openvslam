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
#include <unistd.h>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <exception>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include<opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include"../../../include/System.h"
#include <tuple>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

using namespace std;

const string settingsFile = "/home/meditab/Desktop/orb_slam_ros_required_files/TUM1.yaml";
const string vocab = "/home/meditab/catkin_ws/src/map_save_load-master/ORB_SLAM2/Vocabulary/ORBvoc.txt";

int mode;
bool bLocalizationMode;
bool bmode;
bool bReuse;
int cond;
bool blocali;


class ImageGrabber
{

private:
    cv::Mat imRGB,imD;
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    //void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;

    double roll, pitch, yaw;
    bool val;
    ros::Publisher pos_pub, robotYaw, slam_pitch, Tcw_pos;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_Map_Save");
    ros::start();


    cout << "What is your mode of operation-----------------Shubham?" << endl;
    cout <<   "      1--New Map" << endl;
    cout <<   "      2--Old Map Localization" << endl;

    cin >> cond;
    if(cond==1){
        bReuse=false;
        blocali=false;
    }
    else{
        bReuse=true;
        blocali=true;
    }
    cout<<"OK."<<endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(vocab, settingsFile, ORB_SLAM2::System::RGBD, true, bReuse, argv[1], blocali);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/RGBD_Image", 1, &ImageGrabber::GrabImage,&igb);
    //ros::Publisher poseStampedPub = n.advertise<geometry_msgs::PoseStamped>("/cur_pos", 2, true);
    igb.pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/slam/pos", 1);
    igb.slam_pitch = nh.advertise<std_msgs::Float64>("/orb/pitch", 100);
    igb.Tcw_pos = nh.advertise<geometry_msgs::PoseStamped>("/orb/pos", 100);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveXZ();

    ros::shutdown();

    return 0;
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

    cv::Mat temp = cv_ptr->image;
    try {
        imRGB = temp(cv::Rect(0,0,640,480));
    }
    catch (exception& e)
    {
        cout<<"RGB Error."<<endl;
    }

    cv::Mat bgrCombine;
    try {
        bgrCombine = temp(cv::Rect(640,0,640,480));
    }
    catch(exception& e){
        cout<<"Depth Error."<<endl;
    }

    cv::Mat bgr[3];
    split(bgrCombine , bgr);

    imD = bgr[2];

    tuple <cv::Mat,cv::Mat> curPos = mpSLAM->TrackRGBD(imRGB, imD, cv_ptr->header.stamp.toSec());
    cv::Mat pose = get<0>(curPos);
    cv::Mat traj = get<1>(curPos);

    if(pose.empty() || traj.empty())
    {}
    else
    {
        cv::Mat Rwc = pose.rowRange(0,3).colRange(0,3).t();
        cv::Mat tcw = pose.rowRange(0,3).col(3);
        cv::Mat Ow = -Rwc*tcw;

        /// atan gives output between -90 & +90
        /// atan2 gives output between -180 & +180
        double roll = atan2(Rwc.at<float>(2,1),Rwc.at<float>(2,2)) * 57.2737 ;
        double yaw = atan2(Rwc.at<float>(1,0),Rwc.at<float>(0,0)) * 57.2737 ;
        double pitch = atan(Rwc.at<float>(2,0)/Rwc.at<float>(0,0)) *57.2737;

        /// 2nd Quadrant
        if(Rwc.at<float>(0,0) < 0 && Rwc.at<float>(2,0) >0){
            pitch=pitch+180;
        }

        /// 3rd Quadrant
        if(Rwc.at<float>(0,0) < 0 && Rwc.at<float>(2,0) <0){
            pitch=pitch+180;
        }

        /// 4th Quadrant
        if(Rwc.at<float>(0,0) > 0 && Rwc.at<float>(2,0) <0){
            pitch=pitch+360;
        }


        std_msgs::Float64 pitchVal;
        pitchVal.data = (pitch);

        geometry_msgs::PoseStamped p;
        p.header.frame_id = "my_map";

        p.pose.position.x = traj.at<float>(0,0); //0;//Ow.at<float>(0,0);              //rh_cameraTranslation[0];
        p.pose.position.y = traj.at<float>(1,0); // 0;//Ow.at<float>(1,0);                                //rh_cameraTranslation[1];
        p.pose.position.z =  traj.at<float>(2,0); //0;//Ow.at<float>(2,0);                                  //rh_cameraTranslation[2];
        p.pose.orientation.x = traj.at<float>(3,0); //0;//v[0];//Ow.at<float>(3);
        p.pose.orientation.y = traj.at<float>(4,0); //0;//v[1];//Ow.at<float>(4);
        p.pose.orientation.z = traj.at<float>(5,0); //0;//v[2];//Ow.at<float>(5);
        p.pose.orientation.w = traj.at<float>(6,0); //0;//v[3];//Ow.at<float>(6);


        geometry_msgs::PoseStamped tc;

        tc.header.frame_id = "my_map";
        //p.header.stamp=header.stamp.toSec();
        //setprecision(9)
        tc.pose.position.x = Ow.at<float>(0, 0); //0;//Ow.at<float>(0,0);              //rh_cameraTranslation[0];
        tc.pose.position.y = Ow.at<float>(1, 0); // 0;//Ow.at<float>(1,0);                                //rh_cameraTranslation[1];
        tc.pose.position.z =  Ow.at<float>(2, 0); //0;//Ow.at<float>(2,0);                                  //rh_cameraTranslation[2];
        tc.pose.orientation.x = traj.at<float>(3,0);  //traj.at<float>(3,0); //0;//v[0];//Ow.at<float>(3);
        tc.pose.orientation.y = traj.at<float>(4,0); //0;//v[1];//Ow.at<float>(4);
        tc.pose.orientation.z = traj.at<float>(5,0); //0;//v[2];//Ow.at<float>(5);
        tc.pose.orientation.w = traj.at<float>(6,0); //0;//v[3];//Ow.at<float>(6);

        slam_pitch.publish(pitchVal);
        pos_pub.publish(p);
        Tcw_pos.publish(tc);
    }


//    // Rotation
//    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
//
//    // translation
//    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
//
//    // rotation inverse
//    cv::Mat Rwc = Rcw.t();
//    cv::Mat Ow = -Rwc*tcw;
//
//    geometry_msgs::PoseStamped poseStamped;
//    poseStamped.header.frame_id = "/frame_id_1";
//    poseStamped.header.stamp = ros::Time::now();
}



