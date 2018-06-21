/**
 * @file   RoadMap.cpp
 * @author Deron
 * @date   19/06/2018
 * @brief  generate road map for 
 *         all the frontiers
 *
 * @copyright
 * Copyright (C) 2018.
 */
#include "roadmap/RoadMap.h"

RoadMap::RoadMap(ros::NodeHandle n):
    m_handle(n),
    m_octree_initialize(false)
{
    m_full_map_sub = m_handle.subscribe("/octomap_full", 1, &RoadMap::bin_map_callback, this);
    m_pub_path_marker = m_handle.advertise<visualization_msgs::Marker>("/path", 1);
}

RoadMap::~RoadMap()
{}

void RoadMap::frontier_detector(int depth)
{}

void RoadMap::bin_map_callback(const octomap_msgs::Octomap::ConstPtr &map)
{
    // convert msg to octree
    m_abs_octree.reset(octomap_msgs::fullMsgToMap(*map));
    m_octree = boost::static_pointer_cast<octomap::OcTree>(m_abs_octree);
    if (m_octree != NULL && m_abs_octree != NULL){
        m_octree_initialize = true;
    }else {
        ROS_INFO_STREAM("octomap_msgs::fullMsgToMap failed!");
        exit(0);
    }

    // bound the state space
    double metric_min_x, metric_min_y, metric_min_z;
    double metric_max_x, metric_max_y, metric_max_z;
    m_octree->getMetricMin(metric_min_x, metric_min_y, metric_min_z);
    m_octree->getMetricMax(metric_max_x, metric_max_y, metric_max_z);
    m_tree_depth = m_octree->getTreeDepth();
    m_space.reset(new ompl::base::RealVectorStateSpace(3));
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, metric_min_x);
    bounds.setLow(1, metric_min_y);
    bounds.setLow(2, metric_min_z);
    bounds.setHigh(0, metric_max_x);
    bounds.setHigh(1, metric_max_y);
    bounds.setHigh(2, metric_max_z);
    m_space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    // construct the information space
    m_si.reset(new ompl::base::SpaceInformation(m_space));
    boost::function<bool(const ompl::base::State*)> valid_fn = [&](const ompl::base::State* s) {
        return isStateValid(s);
    };
    m_si->setStateValidityChecker(valid_fn);
    m_si->setStateValidityCheckingResolution(0.2);
    // visualize some information
    ROS_INFO_STREAM("the metric min:"
                    << metric_min_x << ","
                    << metric_min_y << ","
                    << metric_min_z <<",");
    ROS_INFO_STREAM("the metric max:"
                    << metric_max_x << ","
                    << metric_max_y << ","
                    << metric_max_z <<",");

    generate_roadmap();
}

bool RoadMap::isStateValid(const ompl::base::State *state)
{
    const ompl::base::RealVectorStateSpace::StateType* state3D =
        state->as<ompl::base::RealVectorStateSpace::StateType>();
    double x = state3D->values[0];
    double y = state3D->values[1];
    double z = state3D->values[2];
    octomap::point3d coor(x, y, z);
    octomap::OcTreeNode* node = m_octree->search(coor);
    if(node != NULL){
        //ROS_INFO_STREAM("THE NODE VALUE" << node->getLogOdds());
        if(node->getLogOdds() > 0.0){
            return false;
        }else{
            return true;
        }
    }else{
        return false;
    }

//    return true;
}

bool RoadMap::generate_roadmap()
{
    // define the planning problem and set start and goal state
    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(m_si));
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(m_space), goal(m_space);
    start->values[0] = 0.0;
    start->values[1] = 0.0;
    start->values[2] = -10.0;
    goal->values[0] = -40.0;
    goal->values[1] = 30.0;
    goal->values[2] = -5.0;
    pdef->setStartAndGoalStates(start, goal);

    // define the planner and set up the problem
    ompl::base::PlannerPtr planner(new ompl::geometric::RRTstar(m_si));
    planner->setProblemDefinition(pdef);

    while(true){
        // slove the ploblem
        ompl::base::PlannerStatus status = planner->solve(1.0);
        if(status){
            ompl::base::PathPtr path = planner->getProblemDefinition()->getSolutionPath();
            ROS_INFO_STREAM("prblem sloved!");
            path->print(std::cout);
            //path->as<ompl::geometric::PathGeometric>()->printAsMatrix(std::cout);
            boost::shared_ptr<ompl::geometric::PathGeometric> visual_path =
                    boost::static_pointer_cast<ompl::geometric::PathGeometric>(path);
            m_pub_path_marker.publish(ca::planning_common::visualization_utils::GetMarker(*visual_path, 0.1));
        }else{
            ROS_INFO_STREAM("failed!");
        }
        sleep(1);
    }

}


