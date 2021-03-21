#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


void imageCallback(const sensor_msgs::ImageConstPtr &msg){		// 图像回调函数，用于显示图像
    try{
        cv::Mat img = cv_bridge::toCvShare(msg, "mono8")->image;
        imshow("img", img);
        cv::waitKey(1);
    }catch(cv_bridge::Exception &e){
        ROS_ERROR("image callback error");
    }
}

int main(int argc, char **argv){
    //node init
    ros::init(argc, argv, "subscriber_test");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("event_image", 10, imageCallback);	// 采用 回调方式接收图片

    ros::Rate r(30);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}