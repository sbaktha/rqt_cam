// #include <ros/ros.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <sensor_msgs/Image.h>
// #include <image_transport/image_transport.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <sstream>

// using namespace cv;

// static const std::string TOPIC_NAME = "econ_cam/rgb/image";

// int main(int argc, char ** argv){
//     if(argv[1] == NULL) return 1;

//     ros::init(argc, argv, "econ_image_publisher");
//     ros::NodeHandle nh;

//     image_transport::ImageTransport it(nh);
//     image_transport::Publisher pub = it.advertise("camera/image", 1);

//     int video_source;
//     std::istringstream video_sourceCmd(argv[1]);

//     // ros::ROS_INFO_STREAM("Video source command:%s"<<video_sourceCmd<<std::endl);
//     if(!(video_sourceCmd >> video_source)) return -1;

//     cv::VideoCapture cap(video_source);

//     if(!cap.isOpened()) return 1;
//     cv::Mat frame;
//     sensor_msgs::ImagePtr msg;

//     ros::Rate rate(10);
//     while(nh.ok()){
//         cap >> frame;
//         if(!frame.empty()){
//             msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
//             pub.publish(msg);
//             cv::waitKey(1);
//         }

//         ros::spinOnce();
//         rate.sleep();
//     }
//     return 0;
// }

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "image_publisher");
//   ros::NodeHandle nh;
//   image_transport::ImageTransport it(nh);
//   image_transport::Publisher pub = it.advertise("camera/image", 1);

//   cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);
//   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

//   ros::Rate loop_rate(5);
//   while (nh.ok()) {
//     pub.publish(msg);
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    std::istringstream src_arg(argv[1]);
    int video_source;
    if (!(src_arg >> video_source))
        return 1;

    cv::VideoCapture cap(video_source);
    if (!cap.isOpened())
        return 1;

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate rate(10);
    while (nh.ok())
    {
        cap >> frame;
        if (!frame.empty())
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        }

        ros::spinOnce();
        rate.sleep();
    }
}
