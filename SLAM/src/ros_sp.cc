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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

cv::Mat RGB;
cv::Mat DEPTH;
ros::Time rgbTime;
bool newImgFlg=false;
using namespace std;

ros::Publisher CamPose_Pub;
ros::Publisher Camodom_Pub;
ros::Publisher odom_pub;

geometry_msgs::PoseStamped Cam_Pose;
geometry_msgs::PoseWithCovarianceStamped Cam_odom;

cv::Mat Camera_Pose;
tf::Transform orb_slam;
tf::TransformBroadcaster * orb_slam_broadcaster;
std::vector<float> Pose_quat(4);
std::vector<float> Pose_trans(3);

ros::Time current_time, last_time;
double lastx=0,lasty=0,lastth=0;

void Pub_CamPose(cv::Mat &pose);

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SP");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD, true, false, false, false, false); //viewer, localmap, loop, bow, trackonly

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    
    CamPose_Pub = nh.advertise<geometry_msgs::PoseStamped>("/Camera_Pose",1);
    Camodom_Pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/Camera_Odom", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);

    while( ros::ok() )
    {
        ros::spinOnce();
        if(!newImgFlg)
		{
		   continue;
		}
        Pub_CamPose(Camera_Pose); 
		newImgFlg=false;
    }
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("/home/tsui/KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

inline bool file_exist (const std::string& name) {
    ifstream f(name.c_str());
    return f.good();
}

int GetFiles(char** file_list, const char* dirname)
{
    DIR * dir;
    struct dirent * ptr;
    int i=0;
    dir = opendir(dirname); //打开一个目录
    while((ptr = readdir(dir)) != NULL) //循环读取目录数据
    {
    //printf("d_name : %s\n", ptr->d_name); //输出文件名
    strcpy(file_list[i],ptr->d_name ); //存储到数组
    if ( ++i>=5 ) break;
    }
    closedir(dir);//关闭目录指针
    return (i - 2);
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    newImgFlg=true;
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // Get RGB and Depth image
    RGB=cv_ptrRGB->image;
	DEPTH=cv_ptrD->image;
    rgbTime=msgRGB->header.stamp;

    string dir = "/dev/shm/tmpSP/";   //文件夹
    string filename = dir + to_string(rgbTime.toSec()) + ".png";;      //RGB图文件名
    // 读取特征点文件信息

    cv::imwrite(filename, RGB);
    system("touch /dev/shm/tmpSP/ready.txt");
    while( ! file_exist("/dev/shm/tmpSP/done.txt")){
        usleep(5000);
    }
    Camera_Pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    system("rm -r /dev/shm/tmpSP/*");
}

void Pub_CamPose(cv::Mat &pose)
{
    cv::Mat Rwc(3,3,CV_32F);
	cv::Mat twc(3,1,CV_32F);
	Eigen::Matrix<double,3,3> rotationMat;
	orb_slam_broadcaster = new tf::TransformBroadcaster;
	if(pose.dims<2 || pose.rows < 3)
	{
        Rwc = Rwc;
		twc = twc;
	}
	else
	{
		Rwc = pose.rowRange(0,3).colRange(0,3).t();
		twc = -Rwc*pose.rowRange(0,3).col(3);
		
		rotationMat << Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2),
					Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2),
					Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2);
		Eigen::Quaterniond Q(rotationMat);

		Pose_quat[0] = Q.x(); Pose_quat[1] = Q.y();
		Pose_quat[2] = Q.z(); Pose_quat[3] = Q.w();
		
		Pose_trans[0] = twc.at<float>(0);
		Pose_trans[1] = twc.at<float>(1);
		Pose_trans[2] = twc.at<float>(2);
		
		orb_slam.setOrigin(tf::Vector3(Pose_trans[2], -Pose_trans[0], -Pose_trans[1]));
		orb_slam.setRotation(tf::Quaternion(Q.z(), -Q.x(), -Q.y(), Q.w()));
		orb_slam_broadcaster->sendTransform(tf::StampedTransform(orb_slam, ros::Time::now(), "/map", "/base_link"));
		
		Cam_Pose.header.stamp = ros::Time::now();
		Cam_Pose.header.frame_id = "/map";
		tf::pointTFToMsg(orb_slam.getOrigin(), Cam_Pose.pose.position);
		tf::quaternionTFToMsg(orb_slam.getRotation(), Cam_Pose.pose.orientation);
		
		Cam_odom.header.stamp = ros::Time::now();
		Cam_odom.header.frame_id = "/map";
		tf::pointTFToMsg(orb_slam.getOrigin(), Cam_odom.pose.pose.position);
		tf::quaternionTFToMsg(orb_slam.getRotation(), Cam_odom.pose.pose.orientation);
		Cam_odom.pose.covariance = {0.01, 0, 0, 0, 0, 0,
									0, 0.01, 0, 0, 0, 0,
									0, 0, 0.01, 0, 0, 0,
									0, 0, 0, 0.01, 0, 0,
									0, 0, 0, 0, 0.01, 0,
									0, 0, 0, 0, 0, 0.01};
		
		CamPose_Pub.publish(Cam_Pose);
		Camodom_Pub.publish(Cam_odom);
		
		nav_msgs::Odometry odom;
		odom.header.stamp =ros::Time::now();
		odom.header.frame_id = "/map";

		// Set the position
		odom.pose.pose.position = Cam_odom.pose.pose.position;
		odom.pose.pose.orientation = Cam_odom.pose.pose.orientation;

		// Set the velocity
		odom.child_frame_id = "/base_link";
		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		double vx = (Cam_odom.pose.pose.position.x - lastx)/dt;
		double vy = (Cam_odom.pose.pose.position.y - lasty)/dt;
		double vth = (Cam_odom.pose.pose.orientation.z - lastth)/dt;
		
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		// Publish the message
		odom_pub.publish(odom);
		
		last_time = current_time;
		lastx = Cam_odom.pose.pose.position.x;
		lasty = Cam_odom.pose.pose.position.y;
		lastth = Cam_odom.pose.pose.orientation.z;
	}
}


