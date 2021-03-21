#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "celex5/celex5.h"

#define BIN_FILE "/home/yuguanfeng/MipiData_20210302_150548827_FO1_100M.bin"

int main(int argc, char **argv){
    //node init
    ros::init(argc, argv, "publisher_test");
    ros::NodeHandle nh;


    //celex init
    CeleX5 *pCeleX5 = new CeleX5;
    if (pCeleX5 == NULL){
        ROS_ERROR("Can not create celex pointer...");
        return -1;
    }
		
    //pCeleX5->openSensor(CeleX5::CeleX5_MIPI);
	if(!(pCeleX5->openBinFile(BIN_FILE)))
    {
        ROS_ERROR("Can not open the bin file...");
        return -1;
    }
    CeleX5::CeleX5Mode sensorMode = (CeleX5::CeleX5Mode)pCeleX5->getBinFileAttributes().loopAMode;
    uint8_t * pImageBuffer = new uint8_t[CELEX5_PIXELS_NUMBER];		//存储数据，转化为image。这个Number定义为 1280*800

    ROS_INFO("open the bin file...");
    // ROS发布图像数据
    image_transport::ImageTransport it(nh);
    ROS_INFO("image transport...");
    image_transport::Publisher pub = it.advertise("event_image", 10);

    
//cv_bridge::CvImagePtr frame; //不知道用处
    ros::Rate r(30);

    
    while(ros::ok()){    
        
        //p->getEventPicBuffer(buf, CeleX5::EventBinaryPic);      //官方的读取数据的API函数

        if(pCeleX5)
        {
            pCeleX5->readBinFileData();//start reading the bin file
            ROS_INFO("Start reading the bin file...");
        }

        if (CeleX5::Full_Picture_Mode == sensorMode)
	    {
		    //full-frame picture
		    pCeleX5->getFullPicBuffer(pImageBuffer);
		    cv::Mat matFullPic(800, 1280, CV_8UC1, pImageBuffer);
		    cv::imshow("FullPic", matFullPic);
		    //cv::waitKey(1);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", matFullPic).toImageMsg();  // 转化格式
            pub.publish(msg);				// 发布数据
            ros::spinOnce();
            r.sleep();
	    }
	    else if (CeleX5::Event_Off_Pixel_Timestamp_Mode == sensorMode)
	    {
		    //get buffers when sensor works in EventMode
		    pCeleX5->getEventPicBuffer(pImageBuffer, CeleX5::EventBinaryPic);
		    cv::Mat matEventPic(800, 1280, CV_8UC1, pImageBuffer);
		    cv::imshow("Event Binary Pic", matEventPic);
		    //cvWaitKey(1);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", matEventPic).toImageMsg();  // 转化格式
            pub.publish(msg);				// 发布数据
            ros::spinOnce();
            r.sleep();
	    }
	    else if (CeleX5::Optical_Flow_Mode == sensorMode)
	    {
		    //full-frame optical-flow pic
		    pCeleX5->getOpticalFlowPicBuffer(pImageBuffer, CeleX5::OpticalFlowPic);
		    cv::Mat matOpticalFlow(800, 1280, CV_8UC1, pImageBuffer);
		    cv::imshow("Optical-Flow Pic", matOpticalFlow);
		    //cvWaitKey(1);
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", matOpticalFlow).toImageMsg();  // 转化格式
            pub.publish(msg);				// 发布数据
            ros::spinOnce();
            r.sleep();
	    }
        
    }
    return 0;
}