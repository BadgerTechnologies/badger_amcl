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

// Info for a node in the tree
struct PFKDTreeNode
{
  // Depth in the tree
  int leaf, depth;

  // Pivot dimension and value
  int pivot_dim;
  double pivot_value;

  // The key for this node
  int key[3];

  // The value for this node
  double value;

  // The cluster label (leaf nodes)
  int cluster;

  // Child nodes
  struct PFKDTreeNode* children[2];

};

// A kd tree
class PFKDTree
{
public:
  // Create a tree
  PFKDTree();

  // Clear all entries from the tree
  void clearKDTree();

  // Insert a pose into the tree
  void insertPose(const Eigen::Vector3d& pose, double value);

  // Cluster the leaves in the tree
  void cluster();

  // Determine the cluster label for the given pose
  int getCluster(const Eigen::Vector3d& pose);

  int getLeafCount();

private:
  // Compare keys to see if they are equal
  bool equals(int key_a[], int key_b[]);

  // Insert a node into the tree
  PFKDTreeNode* insertNode(PFKDTreeNode* parent, PFKDTreeNode* node, int key[], double value);

  // Recursive node search
  PFKDTreeNode* findNode(PFKDTreeNode* node, int key[]);

  // Recursively label nodes in this cluster
  void clusterNode(PFKDTreeNode* node, int depth);

  // Cell size
  double cell_size_[3];

  // The root node of the tree
  PFKDTreeNode* root_;

  std::deque<PFKDTreeNode> nodes_;

  // The number of leaf nodes in the tree
  int leaf_count_;
};

}  // namespace amcl

#endif  // AMCL_PF_PF_KDTREE_H
