#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <autoware_msgs/image_obj.h>
#include <autoware_msgs/image_obj_tracked.h>
#include <autoware_msgs/image_obj_ranged.h>

#include <frame_publisher/frame_publisher.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/calib3d.hpp>

#include <boost/filesystem.hpp>

#include <sstream>
#include <iostream>

const int WIDTH = 960; //1920
const int HEIGHT = 540; // 1080

const std::string FILENAME ="/home/henry/Downloads/squareone_stopsign.mp4";

const std::string NODE_NAME = "talker";
const std::string WINDOW_NAME = NODE_NAME + ":imageMsg";
const std::string TOPIC_NAME = "/frame_images";

const int PUBLISH_RATE = 10;

void image_obj_tracked_callback(const autoware_msgs::image_obj_tracked & aMsg)
{
    ROS_INFO("Got a tracked object: ");
    for (int i = 0; i < aMsg.rect_ranged.size(); i++) {
        ROS_INFO("i = %d, x = %d, y = %d, width = %d, height = %d",
            i,
            aMsg.rect_ranged[i].rect.x,
            aMsg.rect_ranged[i].rect.y,
            aMsg.rect_ranged[i].rect.width,
            aMsg.rect_ranged[i].rect.height
        ); 
    }      
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, NODE_NAME);


    ros::NodeHandle vNode;

    ros::Subscriber vSubImageObjTracked = vNode.subscribe(
        "/image_obj_tracked",
        1,
        image_obj_tracked_callback
        );


    image_transport::ImageTransport vImageTransport(vNode);
    image_transport::Publisher vImagePublisher(vImageTransport.advertise(TOPIC_NAME.c_str(), 1));
    ROS_INFO("Publishing to topic %s", TOPIC_NAME.c_str());

  
    cv::namedWindow(WINDOW_NAME);
    cv::setWindowProperty(WINDOW_NAME, CV_WND_PROP_FULLSCREEN,CV_WINDOW_FULLSCREEN);
    cv::resizeWindow(WINDOW_NAME, WIDTH, HEIGHT);

    try {
        std::string vImagePath;
        if (! vNode.getParam("/talker/image_directory", vImagePath)) {
            std::cerr << "image_directory parameter not specified!" << std::endl;
            ros::shutdown();
        }

        ROS_INFO("Reading images from %s", vImagePath.c_str());

        ros::Rate vLoopRate(PUBLISH_RATE);

        boost::filesystem::path vPath(vImagePath);
        std::vector<std::string> vPaths;
        if (boost::filesystem::exists(vPath) && boost::filesystem::is_directory(vPath)) {
            for (auto& file : boost::filesystem::directory_iterator(vPath)) {
                vPaths.push_back(file.path().string());
            }

            std::sort(vPaths.begin(), vPaths.end());

            for (auto& vPath : vPaths) {
                cv::Mat vFrame = cv::imread(vPath, CV_LOAD_IMAGE_COLOR);
                
                cv_bridge::CvImage vCvImage = cv_bridge::CvImage(
                    std_msgs::Header(),
                    "bgr8",
                    vFrame
                    );
                sensor_msgs::Image vImageMsg;
                vCvImage.toImageMsg(vImageMsg);
                vImagePublisher.publish(vImageMsg);

                cv::Mat vFrameResized;
                cv::resize(vFrame, vFrameResized, cv::Size(WIDTH, HEIGHT), 0, 0, CV_INTER_LINEAR);
    
                //cv::imshow(WINDOW_NAME, vFrameResized);
                //cv::waitKey(1);

                
                ros::spinOnce();
                vLoopRate.sleep();
            }
        }

        ROS_INFO("Done publishing raw images");
        
        cv::destroyAllWindows();
        ros::spin();

    } catch (std::string &e) {
        std::cerr << e << std::endl;
        ros::shutdown();
    }
    
    return 0;
}
