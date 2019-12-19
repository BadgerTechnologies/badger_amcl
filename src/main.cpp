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
 *  Maintainter: Tyler Buchman (tyler_buchman@jabil.com)
 *
 */

#include <boost/shared_ptr.hpp>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/spinner.h>
#include <signal.h>

#include <iostream>

#include "node/node.h"

using namespace amcl;

boost::shared_ptr<Node> amcl_node_ptr;
bool sigFlag = false;

void sigHandler(int sig)
{
  std::cout << "Sig int detected, ros shutting down.\n";
  sigFlag = true;
  ros::shutdown();
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "amcl", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  // Override default sigint handler
  signal(SIGINT, sigHandler);
  signal(SIGTERM, sigHandler);

  // Make our node available to sigintHandler
  amcl_node_ptr.reset(new Node());

  // Uncomment for single threaded
  //ros::spin();

  // Uncomment for multithreaded
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  amcl_node_ptr->savePoseToFile();

  // Without this, our boost locks are not shut down nicely
  amcl_node_ptr.reset();

  // To quote Morgan, Hooray!
  return(0);
}
