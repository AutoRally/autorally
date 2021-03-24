#ifndef ARUCODETECTIONS_H
#define ARUCODETECTIONS_H

#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

namespace autorally_estimation {

    // aruco _marker
    struct ArucoDetection {
        double size;
        int id;
        std::vector<float> detections;
    };

    // aruco_msg
    struct ArucoDetections {
        // _header_type header;
        // const _header_type header;
        // Header_<ContainerAllocator> header;
        // Type header;
        // std_msgs::Type header;
        // std_msgs::Header_ header;
        // std_msgs::Header_<std_msgs::ContainerAllocator> header;
        // sensor_msgs::ImageConstPtr header;
        // const std_msgs::Header_<std::allocator<void>> header;
        std_msgs::Header_<std::allocator<void>> header;


        std::vector<autorally_estimation::ArucoDetection> detections;
    };

    // const _header_type {aka const std_msgs::Header_<std::allocator<void> >}’)
}

#endif