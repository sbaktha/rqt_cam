#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string.h>
#include <sstream>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher *pub;

    std::istringstream src_arg(argv[1]);
    int cam_count = 0;
    if (!(src_arg >> cam_count))
        return 1;

    cv::VideoCapture *cap;
    char* topic;

    for (int i = 0; i < cam_count; i++)
    {
        topic = std::strcat(std::strcat("camera_", std::to_string(i).c_str()), "/image");
        pub[i] = it.advertise(topic, 1);
        cap[i] = cv::VideoCapture(i);
        if (!(cap[i].isOpened()))
            return 1;
    }

    cv::Mat *frame;
    sensor_msgs::ImagePtr *msg;

    ros::Rate rate(10);
    while (nh.ok())
    {
        for (int i = 0; i < cam_count; i++)
        {
            cap[i] >> frame[i];
            if (!frame[i].empty())
            {
                msg[i] = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame[i]).toImageMsg();
                pub[i].publish(msg[i]);
                cv::waitKey(1);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}
