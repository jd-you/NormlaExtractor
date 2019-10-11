#include "normal_extractor.h"

#include <ros/ros.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <normal_extractor/NEXTServerConfig.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "normal_extractor_node");

    Planning3D::NormalExtractor ne;

    // dynamic reconfigure
    dynamic_reconfigure::Server<normal_extractor::NEXTServerConfig> server;
    dynamic_reconfigure::Server<
        normal_extractor::NEXTServerConfig>::CallbackType f;
    f = boost::bind(&Planning3D::NormalExtractor::dynCallback, &ne, _1, _2);
    server.setCallback(f);

    ros::Rate rate(1);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
