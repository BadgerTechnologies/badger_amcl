/*
 *
 *       AMCL Octomap Class
 *        by Tyler Buchman
 *        2019
 *
 */


#include "map.h"
#include "octomap.h"
#include "amcl_odom.h"
#include "amcl_node.h"
#include "amcl/AMCLConfig.h"

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"

using namespace amcl;

void
AmclNode::init3D()
{
}

void
AmclNode::deleteAmclNode3D()
{
}

void
AmclNode::globalLocalizationCallback3D()
{
}

void
AmclNode::initFromNewMap3D()
{
}

void
AmclNode::freeMapDependentMemory3D()
{
}

void
AmclNode::reconfigure3D(amcl::AMCLConfig &config)
{
}

OctoMap*
AmclNode::convertMap(const sensor_msgs::PointCloud& map_msg)
{
  return new OctoMap();
}

double
AmclNode::scorePose3D(const PFVector &p)
{
  return 0.0;
}
