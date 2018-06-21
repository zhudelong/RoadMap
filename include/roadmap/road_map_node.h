/**
 * @file   road_map_node.h
 * @author Deron
 * @date   19/06/2018
 * @brief  set up ros node
 *
 * @copyright
 * Copyright (C) 2018.
 */

#ifndef _ROAD_MAP_NODE_H_
#define _ROAD_MAP_NODE_H_

#include <ros/ros.h>
//// get depth and query happens in a shellow level
//int tree_depth = m_octree->getTreeDepth();
//// traverse the whole tree
//int node_number = 0;
//for(octomap::OcTree::tree_iterator it = m_octree->begin_tree(tree_depth-5),
//    end=m_octree->end_tree(); it!= end; ++it)
//{
//  //manipulate node, e.g.:
////      ROS_INFO_STREAM("Node center: " << it.getCoordinate());
////      ROS_INFO_STREAM("Node size: " << it.getSize());
////      ROS_INFO_STREAM("Node value: " << it->getValue());
//    node_number ++;
//}
//ROS_INFO_STREAM("the logodds value is:" << node_number);


#endif /* _ROAD_MAP_NODE_H_ */
