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
  // Testing pdf gaussian sample function with diagonal covariance matrix
  Eigen::Vector3d x(1, 1, 1);
  Eigen::Matrix3d cx;
  cx << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  badger_amcl::PDFGaussian pdf_gaussian(x, cx);
  Eigen::Vector3d sample = pdf_gaussian.sample();
  EXPECT_DOUBLE_EQ(sample[0], 0.26562654174915334);
  EXPECT_DOUBLE_EQ(sample[1], 0.97172090090793528);
  EXPECT_DOUBLE_EQ(sample[2], -1.5856194295513539);
  // Testing pdf gaussian sample function with non-diagonal covariance matrix
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
  badger_amcl::PFKDTree pf_kdtree(Eigen::Vector3d(0.5, 0.5, 10 * M_PI / 180));
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

TEST(TestBadgerAmcl, testOctoMapConversions)
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

TEST(TestBadgerAmcl, testOccupancyMapConversions)
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

TEST(TestBadgerAmcl, testOccupancyMapDistances)
{
  double resolution = 0.05;
  badger_amcl::OccupancyMap occupancy_map(resolution);
  std::vector<int> size_vec = {100, 150};
  occupancy_map.setSize(size_vec);
  EXPECT_EQ(occupancy_map.getSize(), size_vec);
  double x_origin, y_origin;
  x_origin = size_vec[0] / 2 * resolution;
  y_origin = size_vec[1] / 2 * resolution;
  occupancy_map.setOrigin(pcl::PointXYZ(x_origin, y_origin, 0.0));
  for (int x = 0; x < size_vec[0]; x++)
  {
    for (int y = 0; y < size_vec[1]; y++)
    {
      unsigned int index = occupancy_map.computeCellIndex(x, y);
      badger_amcl::MapCellState state;
      if (x == 1 and y > 2 and y < 12)
        state = badger_amcl::MapCellState::CELL_UNKNOWN;
      else if (x > 4 and x < 14 and (y == 10 or y == 15))
        state = badger_amcl::MapCellState::CELL_OCCUPIED;
      else
        state = badger_amcl::MapCellState::CELL_FREE;
      occupancy_map.setCellState(index, state);
      EXPECT_EQ(occupancy_map.getCellState(x, y), state);
    }
  }
  EXPECT_TRUE(occupancy_map.isValid({0, 0}));
  EXPECT_TRUE(not occupancy_map.isValid({-1, 5}));
  EXPECT_TRUE(occupancy_map.isValid({99, 149}));
  EXPECT_TRUE(not occupancy_map.isValid({100, 150}));
  EXPECT_TRUE(not occupancy_map.isValid({149, 99}));
  occupancy_map.updateDistancesLUT(0.3);
  EXPECT_EQ(occupancy_map.getCellState(0, 0), badger_amcl::MapCellState::CELL_FREE);
  EXPECT_EQ(occupancy_map.getCellState(1, 3), badger_amcl::MapCellState::CELL_UNKNOWN);
  EXPECT_EQ(occupancy_map.getCellState(5, 10), badger_amcl::MapCellState::CELL_OCCUPIED);
  double range = occupancy_map.calcRange(0, 0, 0, 0);
  EXPECT_DOUBLE_EQ(range, 0.0);
  range = occupancy_map.calcRange(0.05, 0, 1.5708, 0.5);
  EXPECT_DOUBLE_EQ(range, 0.15);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "BadgerAMCLTests");
  auto v = RUN_ALL_TESTS();
  return v;
}
