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

#include <memory>
#include <deque>

#include <Eigen/Dense>

namespace badger_amcl
{

struct PFKDTreeNode
{
  int depth;
  int pivot_dim;
  int key[3];
  double value;
  int cluster;
  struct PFKDTreeNode* children[2];
};

class PFKDTree
{
public:
  PFKDTree();
  void clearKDTree();
  void insertPose(const Eigen::Vector3d& pose, double value);
  void cluster();
  int getCluster(const Eigen::Vector3d& pose);
  int getLeafCount();

private:
  bool equals(int key_a[], int key_b[]);
  PFKDTreeNode* insertNode(PFKDTreeNode* node, int key[], double value, int depth);
  PFKDTreeNode* makeLeafNode(int key[], double value, int depth);
  void traverseNode(PFKDTreeNode* node, int key[], double value, int depth);
  PFKDTreeNode* findNode(PFKDTreeNode* node, int key[]);
  void clusterNode(PFKDTreeNode* node);

  double cell_size_[3];
  PFKDTreeNode* root_;
  std::deque<PFKDTreeNode> nodes_;
  int leaf_count_;
};

}  // namespace badger_amcl

#endif  // AMCL_PF_PF_KDTREE_H
