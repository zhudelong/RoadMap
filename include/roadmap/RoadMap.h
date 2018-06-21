/**
 * @file   RoadMap.h
 * @author Deron
 * @date   19/06/2018
 * @brief  generate road map for 
 *         all the frontiers
 *
 * @copyright
 * Copyright (C) 2018.
 */

#ifndef _ROAD_MAP_H_
#define _ROAD_MAP_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <eigen3/Eigen/Eigen>
#include "planning_common/utils/visualization_utils.h"
//namespace ob = ompl::base;
//namespace og = ompl::geometric;

class RoadMap{
/* node initialization
   including specify the node input and output*/
public:
    RoadMap(ros::NodeHandle n);
    ~RoadMap();
    bool init_pub_sub();
    void frontier_detector(int depth);
    void bin_map_callback(const octomap_msgs::Octomap::ConstPtr &map);
private:
    ros::NodeHandle m_handle;
    ros::Subscriber m_full_map_sub;
    ros::Subscriber m_bin_map_sub;
    ros::Subscriber m_plan_goal_sub;
    ros::Publisher m_pub_path_marker;
    ros::ServiceServer m_roadmap_service;

/* octomap matainer
   matatins a octotree for query inside roadmap node*/
public:
    bool get_value(const Eigen::Vector3d &query, int &value);
    bool get_value(const Eigen::Vector3d &query, double &value);
    bool isStateValid(const ompl::base::State *state);

private:
    boost::shared_ptr<octomap::AbstractOcTree> m_abs_octree;
    boost::shared_ptr<octomap::OcTree> m_octree;
    bool m_octree_initialize;
    int m_tree_depth;
    int m_query_depth;

/* road map generation
   generate cost for all the plan goals*/
public:
    bool generate_roadmap();
    void visualize_roadmap();

private:
    ompl::base::StateSpacePtr m_space;
    ompl::base::SpaceInformationPtr m_si;

};

#endif /* _ROAD_MAP_H_ */
