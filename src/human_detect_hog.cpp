#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2//core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <strings.h>
#include <iostream>
#include <unistd.h>
#include <vector>
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/Float32MultiArray.h>
//#include <people_msgs/PeopleAnglesHog.h>


#define AngleofView 58

class depth_estimater
{
public:
    depth_estimater();
    ~depth_estimater();
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::NodeHandle nh;
    visualization_msgs::Marker marker;
    std_msgs::Float32MultiArray array;
    std::vector<double>angle;
    ros::Subscriber sub_rgb, sub_depth;
    ros::Publisher marker_pub;
    ros::Publisher array_pub;
};

depth_estimater::depth_estimater()
{
    sub_rgb = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 1, &depth_estimater::rgbImageCallback, this);
    sub_depth = nh.subscribe<sensor_msgs::Image>("/camera/depth_registered/image_raw", 1, &depth_estimater::depthImageCallback, this);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    array_pub = nh.advertise<std_msgs::Float32MultiArray>("human_angle", 1);
}
depth_estimater::~depth_estimater(){}

cv::Mat mat_resizer(cv::Mat in_mat, int row, int col)
{
    cv::Mat tmp;
    tmp = cv::Mat::ones(in_mat.rows * row / in_mat.rows, in_mat.cols * col / in_mat.cols, CV_32FC1);
    cv::resize(in_mat, tmp, tmp.size(), cv::INTER_CUBIC);
    return tmp;
}

//RGB
void depth_estimater::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr rgb_ptr;
    ros::Rate r(10);

    try
    {
        rgb_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& ex)
    {
        ROS_ERROR("error");
        exit(-1);
    }
    cv::Mat rgb_im = rgb_ptr->image;

    // 画像リサイズ
    cv::Mat img_resized;
    cv::resize(rgb_im, img_resized, cv::Size(), 0.4, 0.4);

    cv::HOGDescriptor hog;
    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    std::vector<cv::Rect> found;
    hog.detectMultiScale(img_resized, found, 0.2, cv::Size(8,8), cv::Size(16,16), 1.01, 2);
    std::vector<cv::Rect>::const_iterator it = found.begin();

    double rect_center_x = 0.0;
    double theta = 0.0;

    int8_t human_num = found.size();
    //std::vector<double>angle;
    std::vector<double>:: iterator itr;

    array.data.clear();
    angle.clear();
    for(; it!=found.end(); ++it)
    {
        cv::Rect r = *it;
        // 描画に際して，検出矩形を若干小さくする
        r.x += cvRound(r.width*0.1);
        r.width = cvRound(r.width*0.8);
        r.y += cvRound(r.height*0.07);
        r.height = cvRound(r.height*0.8);
        rect_center_x = cvRound((r.x+(r.x+r.width))/2);
        cv::rectangle(img_resized, r.tl(), r.br(), cv::Scalar(0,255,0), 3);
        theta = AngleofView-rect_center_x/img_resized.cols*AngleofView;
        //Convert theta to the angle from center
        angle.push_back(theta-AngleofView/2);
        array.data.push_back(theta-AngleofView/2);
    }
    array_pub.publish(array);

    for(itr = angle.begin();itr != angle.end();itr++)
 	{
   		std::cout << "found:" << found.size() << std::endl;
   		std::cout << "angle:" << *itr << std::endl;

              //Visualization human angle by arrow
              marker.header.frame_id = "/laser";
              marker.header.stamp = ros::Time::now();
              marker.ns = "basic_shapes";
              marker.type = visualization_msgs::Marker::ARROW;
              marker.action = visualization_msgs::Marker::ADD;
               // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
               marker.pose.orientation=tf::createQuaternionMsgFromYaw(*itr/180.0*M_PI);

               // Set the scale of the marker -- 1x1x1 here means 1m on a side
               marker.scale.x = 1.0;
               marker.scale.y = 0.1;
               marker.scale.z = 0.1;
               // Set the color -- be sure to set alpha to something non-zero!
               marker.color.r = 0.0f;
               marker.color.g = 1.0f;
               marker.color.b = 0.0f;
               marker.color.a = 1.0;

              marker.lifetime = ros::Duration();
              marker_pub.publish(marker);


              r.sleep();
            // Publish the marker
            //marker_pub.publish(marker);
             }

    if (found.size()==0)
    {
    	std::cout << "found:" << found.size() << std::endl;
    }

    // 結果の描画
    cv::namedWindow("result", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
    cv::imshow( "result", img_resized );
    cv::waitKey(10);

}

//Depth
void depth_estimater::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr depth_ptr;

    try
    {
        depth_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch(cv_bridge::Exception& ex)
    {
        ROS_ERROR("error");
        exit(-1);
    }

    cv::Mat depth_im = depth_ptr -> image;
    cv::Mat  depth_im_norm = depth_im.clone();
    cv::normalize(depth_im_norm, depth_im_norm, 1, 0, cv::NORM_MINMAX);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "human_detect_hog");
    depth_estimater depth_estimater;
    ros::spin();
    return 0;
}
