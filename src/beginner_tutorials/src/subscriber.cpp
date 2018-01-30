#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <autoware_msgs/image_obj.h>
#include <autoware_msgs/image_obj_tracked.h>
#include <autoware_msgs/image_obj_ranged.h>

#include <frame_publisher/frame_publisher.h>

#include <sstream>
#include <opencv2/opencv.hpp>
#include <iostream>

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
 
  ros::init(argc, argv, "subscriber");

    ros::NodeHandle vNode;
    ros::Subscriber vSubImageObjTracked = vNode.subscribe(
        "/image_obj_tracked",
        1,
        image_obj_tracked_callback
        );
  ros::spin();

  return 0;
}