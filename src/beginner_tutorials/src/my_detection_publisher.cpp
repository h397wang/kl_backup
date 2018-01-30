#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <autoware_msgs/image_obj.h>
#include <autoware_msgs/image_obj_tracked.h>
#include <autoware_msgs/image_obj_ranged.h>

//#include <detections_publisher/detections_publisher.h>
#include <wato_common/FrameDetections.h>
#include <wato_common/detection_label.h>


#include <sstream>
#include <opencv2/opencv.hpp>
#include <iostream>


#include <autoware_msgs/image_obj.h>
#include <autoware_msgs/image_obj_tracked.h>
#include <autoware_msgs/image_obj_ranged.h>

const std::string NODE_NAME = "my_detection_publisher";

const std::string TOPIC_NAME = "/frame_obj_detections";

const int PUBLISH_RATE = 10;

const int LABELS_TO_SKIP = 10; // Publish every LABELS_TO_SKIP frames 

int main(int argc, char **argv)
{

    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle vNode;

    ros::Publisher vPubDetection = vNode.advertise<autoware_msgs::image_obj_ranged>(TOPIC_NAME, 1);
    autoware_msgs::image_obj_ranged vImageObjRanged;

    try {
        std::string vFilePath;
        if (! vNode.getParam("/my_detection_publisher/detections_file", vFilePath)) { // not reading the param properlyfor some reason
            throw std::string("detections_file parameter not specified!");
        }
        std::ifstream vFile(vFilePath.c_str(), std::ios::in);
        if (vFile.fail()) {
           throw std::string("Could not read labels file " + vFilePath);
        }

        ROS_INFO("Reading detections file: %s", vFilePath.c_str());

        ros::Rate loop_rate(PUBLISH_RATE);

        //read line by line
        int vCount = 0;
        std::string vLine;
        while (getline (vFile, vLine)) {

            if (vCount == LABELS_TO_SKIP) {
            
                // Each line in the text file corresponds to a single object in a single frame
                DetectionLabel vLabel(vLine);
            
                autoware_msgs::image_rect_ranged vImageRectRanged;

                vImageRectRanged.rect.x = vLabel.bbox[0];
                vImageRectRanged.rect.y = vLabel.bbox[1];
                vImageRectRanged.rect.width = vLabel.bbox[2] - vLabel.bbox[0];
                vImageRectRanged.rect.height = vLabel.bbox[3] - vLabel.bbox[1];
                vImageRectRanged.rect.score = vLabel.score;

                // TODO
                vImageRectRanged.range;
                vImageRectRanged.min_height;
                vImageRectRanged.max_height;

                vImageObjRanged.obj.push_back(vImageRectRanged);

                vImageObjRanged.header = std_msgs::Header();  
                //vImageObjRanged.header.stamp = ros::Time::now();
                //vImageObjRanged.header.frame_id = "frame_id";
                vImageObjRanged.type = "type";

                // Don't publish if it's garbage
                if (1 
                    && vImageRectRanged.rect.width != 0
                    && vImageRectRanged.rect.height != 0 )
                {

                    vPubDetection.publish(vImageObjRanged);
                    ROS_INFO("Publishing autoware_msgs::image_obj_ranged msg");
                }

                
                vCount = 0;

            } else {
                vCount++;
            }
            
            ros::spinOnce();
            loop_rate.sleep();
        }
    
        ROS_INFO("Done publishing");
        ros::spin();
        
    } catch (std::string &e) {
        std::cerr << e << std::endl;
        ros::shutdown();
    }

    return 0;
}
