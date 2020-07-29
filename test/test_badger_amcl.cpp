/*
 *  Copyright (C) 2020 Badger Technologies, LLC
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

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "map/occupancy_map.h"
#include "map/octomap.h"
#include "pf/pdf_gaussian.h"
#include "pf/pf_kdtree.h"

TEST(TestBadgerAmcl, testPdfGaussian)
{
  // test 1
  Eigen::Vector3d x(1, 1, 1);
  Eigen::Matrix3d cx;
  cx << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  badger_amcl::PDFGaussian pdf_gaussian(x, cx);
  Eigen::Vector3d sample = pdf_gaussian.sample();
  EXPECT_DOUBLE_EQ(sample[0], 0.26562654174915334);
  EXPECT_DOUBLE_EQ(sample[1], 0.97172090090793528);
  EXPECT_DOUBLE_EQ(sample[2], -1.5856194295513539);
  // test 2
  Eigen::Vector3d x2(0, 3, 2);
  Eigen::Matrix3d cx2;
  cx2 << 0.5, 0.1, 0.2, 0.3, 0.6, 0.2, 0.1, 0.7, 0.2, 0.8;
  badger_amcl::PDFGaussian pdf_gaussian2(x2, cx2);
  Eigen::Vector3d sample2 = pdf_gaussian.sample();
  EXPECT_DOUBLE_EQ(sample2[0], 1.6262083813236745);
  EXPECT_DOUBLE_EQ(sample2[1], 1.1142314205031041);
  EXPECT_DOUBLE_EQ(sample2[2], 0.37407538872488655);
}

TEST(TestBadgerAmcl, testPfKdtree)
{
  badger_amcl::PFKDTree pf_kdtree;
  EXPECT_EQ(pf_kdtree.getLeafCount(), 0);
  Eigen::Vector3d pose(1, 1, 1);
  double value = 0.0;
  pf_kdtree.insertPose(pose, value);
  EXPECT_EQ(pf_kdtree.getLeafCount(), 1);
  pf_kdtree.clearKDTree();
  EXPECT_EQ(pf_kdtree.getLeafCount(), 0);
  pf_kdtree.insertPose(pose, value);
  EXPECT_EQ(pf_kdtree.getCluster(pose), -1);
  pf_kdtree.cluster();
  EXPECT_EQ(pf_kdtree.getCluster(pose), 0);
  Eigen::Vector3d pose2(0, 1, 1);
  Eigen::Vector3d pose3(3, 0, 0);
  pf_kdtree.insertPose(pose2, value);
  pf_kdtree.insertPose(pose3, value);
  pf_kdtree.cluster();
  EXPECT_EQ(pf_kdtree.getCluster(pose), 0);
  EXPECT_EQ(pf_kdtree.getCluster(pose2), 1);
  EXPECT_EQ(pf_kdtree.getCluster(pose3), 2);
  EXPECT_EQ(pf_kdtree.getLeafCount(), 2);
  Eigen::Vector3d pose4(0.5, 1, 1);
  pf_kdtree.insertPose(pose4, value);
  pf_kdtree.cluster();
  EXPECT_EQ(pf_kdtree.getCluster(pose), 0);
  EXPECT_EQ(pf_kdtree.getCluster(pose2), 0);
  EXPECT_EQ(pf_kdtree.getCluster(pose3), 1);
  EXPECT_EQ(pf_kdtree.getCluster(pose4), 0);
  EXPECT_EQ(pf_kdtree.getLeafCount(), 2);
}

TEST(TestBadgerAmcl, testOctoMap)
{
  badger_amcl::OctoMap octomap(0.05, false);
  std::vector<int> rtn_vec_map;
  std::vector<double> rtn_vec_world;
  std::vector<int> map_coords_2d = {1, 2};
  std::vector<double> world_coords_2d = {.05, .1};
  rtn_vec_world.resize(2);
  rtn_vec_map.resize(2);
  octomap.convertMapToWorld(map_coords_2d, &rtn_vec_world);
  octomap.convertWorldToMap(world_coords_2d, &rtn_vec_map);
  for (int i = 0; i < world_coords_2d.size(); i++)
  {
    EXPECT_DOUBLE_EQ(world_coords_2d[i], rtn_vec_world[i]);
  }
  EXPECT_EQ(map_coords_2d, rtn_vec_map);
  std::vector<int> map_coords_3d = {3, 5, -1};
  std::vector<double> world_coords_3d {.15, .25, -.05};
  rtn_vec_world.resize(3);
  rtn_vec_map.resize(3);
  octomap.convertMapToWorld(map_coords_3d, &rtn_vec_world);
  octomap.convertWorldToMap(world_coords_3d, &rtn_vec_map);
  for (int i = 0; i < world_coords_3d.size(); i++)
  {
    EXPECT_DOUBLE_EQ(world_coords_3d[i], rtn_vec_world[i]);
  }
  EXPECT_EQ(map_coords_3d, rtn_vec_map);
}

TEST(TestBadgerAmcl, testOccupancyMap)
{
  badger_amcl::OccupancyMap occupancy_map(0.05);
  std::vector<int> rtn_vec_map;
  std::vector<double> rtn_vec_world;
  std::vector<int> map_coords = {1, 2};
  std::vector<double> world_coords = {.05, .1};
  rtn_vec_world.resize(2);
  rtn_vec_map.resize(2);
  occupancy_map.convertMapToWorld(map_coords, &rtn_vec_world);
  occupancy_map.convertWorldToMap(world_coords, &rtn_vec_map);
  for (int i = 0; i < world_coords.size(); i++)
  {
    EXPECT_DOUBLE_EQ(world_coords[i], rtn_vec_world[i]);
  }
  EXPECT_EQ(map_coords, rtn_vec_map);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "BadgerTester");
  auto v = RUN_ALL_TESTS();
  return v;
}
