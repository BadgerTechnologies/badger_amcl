/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/spinner.h>
#include <signal.h>

#include <iostream>

#include "node/node.h"

void sigHandler(int sig)
{
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "amcl", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  // Override default sigint handler
  signal(SIGINT, sigHandler);
  signal(SIGTERM, sigHandler);

  std::unique_ptr<amcl::Node> amcl_node_ptr(new amcl::Node());

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  amcl_node_ptr->savePoseToFile();

  // Without this, our locks are not shut down nicely
  amcl_node_ptr.reset();

  return 0;
}
