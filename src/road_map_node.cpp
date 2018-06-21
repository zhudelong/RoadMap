#include <ros/ros.h>
#include <roadmap/RoadMap.h>
#include "roadmap/road_map_node.h"
int main(int argc, char **argv){
    ros::init(argc, argv, "road_map_node");
    ros::NodeHandle nh("road_map");
    boost::shared_ptr<RoadMap> map_ = boost::make_shared<RoadMap>(nh);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
