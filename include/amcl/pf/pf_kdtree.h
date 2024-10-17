/*
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
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

#ifndef AMCL_PF_PF_KDTREE_H
#define AMCL_PF_PF_KDTREE_H

#include <array>
#include <memory>
#include <deque>

#include <Eigen/Dense>

namespace badger_amcl
{

class PFKDTree
{

private:
  // Note: This code is tightly-coupled to a dimension_count of 3 (e.g. it
  //   uses the type Eigen::Vector3d throughout). However, it is still useful
  //   to have a name in some places to explain where the magic number "3"
  //   is coming from, especially since this is named K-dimension tree.
  static constexpr std::size_t dimension_count = 3;

public:
  using Key = std::array<int, dimension_count>;

  PFKDTree(const Eigen::Vector3d& cell_size);
  void clearKDTree();
  void insertPose(const Eigen::Vector3d& pose, double value);
  void cluster();
  int getCluster(const Eigen::Vector3d& pose);
  int getLeafCount();
  Key getKey(const Eigen::Vector3d& pose);

private:
  struct PFKDTreeNode
  {
    int depth;
    int pivot_dim;
    Key key;
    double value;
    int cluster;
    struct PFKDTreeNode* children[2];
  };

  PFKDTreeNode* insertNode(PFKDTreeNode* node, Key key, double value, int depth);
  PFKDTreeNode* makeLeafNode(Key key, double value, int depth);
  void traverseNode(PFKDTreeNode* node, Key key, double value, int depth);
  PFKDTreeNode* findNode(PFKDTreeNode* node, Key key);
  void clusterNode(PFKDTreeNode* node);

  Eigen::Vector3d cell_size_;
  PFKDTreeNode* root_;
  std::deque<PFKDTreeNode> nodes_;
  int leaf_count_;
};

}  // namespace amcl

#endif  // AMCL_PF_PF_KDTREE_H
