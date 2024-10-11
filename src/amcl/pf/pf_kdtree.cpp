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

#include "pf/pf_kdtree.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <ros/assert.h>

namespace badger_amcl
{

PFKDTree::PFKDTree()
{
  cell_size_[0] = 0.50;
  cell_size_[1] = 0.50;
  cell_size_[2] = (10 * M_PI / 180);
  root_ = NULL;
  leaf_count_ = 0;
}

void PFKDTree::clearKDTree()
{
  root_ = NULL;
  leaf_count_ = 0;
  nodes_.clear();
}

void PFKDTree::insertPose(const Eigen::Vector3d& pose, double value)
{
  Key key = getKey(pose);
  root_ = insertNode(root_, key, value, 0);
}

void PFKDTree::cluster()
{
  PFKDTreeNode* node;
  for (int i = 0; i < nodes_.size(); i++)
  {
    nodes_[i].cluster = -1;
  }

  int cluster_count = 0;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].cluster != -1)
    {
      continue;
    }
    nodes_[i].cluster = cluster_count++;
    clusterNode(&nodes_[i]);
  }
}

// Determine the cluster label for the given pose
int PFKDTree::getCluster(const Eigen::Vector3d& pose)
{
  Key key = getKey(pose);
  PFKDTreeNode* node;
  node = findNode(root_, key);
  if (node == NULL)
    return -1;
  return node->cluster;
}

int PFKDTree::getLeafCount()
{
  return leaf_count_;
}

PFKDTree::PFKDTreeNode* PFKDTree::insertNode(PFKDTreeNode* node, Key key, double value, int depth)
{
  if (node == NULL)
  {
    node = makeLeafNode(key, value, depth);
  }
  else
  {
    if (key == node->key)
    {
      node->value += value;
    }
    else
    {
      traverseNode(node, key, value, depth);
    }
  }
  return node;
}

PFKDTree::PFKDTreeNode* PFKDTree::makeLeafNode(Key key, double value, int depth)
{
  nodes_.push_back(PFKDTreeNode());
  PFKDTreeNode* node = &nodes_.back();
  node->depth = depth;
  node->key = key;
  node->value = value;
  node->pivot_dim = -1;
  node->cluster = -1;
  leaf_count_ += 1;
  return node;
}

void PFKDTree::traverseNode(PFKDTreeNode* node, Key key, double value, int depth)
{
  if (node->pivot_dim == -1)
  {
    int split, max_split = 0;
    for (int i = 0; i < dimension_count; i++)
    {
      split = std::abs(key[i] - node->key[i]);
      if (split > max_split)
      {
        max_split = split;
        node->pivot_dim = i;
      }
    }
    ROS_ASSERT(node->pivot_dim >= 0);
    leaf_count_ -= 1;
  }
  int child = key[node->pivot_dim] > node->key[node->pivot_dim];
  node->children[child] = insertNode(node->children[child], key, value, depth+1);
}

PFKDTree::PFKDTreeNode* PFKDTree::findNode(PFKDTreeNode* node, Key key)
{
  if (node == NULL)
  {
    return NULL;
  }
  else if (key == node->key)
  {
    return node;
  }
  else
  {
    int child = key[node->pivot_dim] > node->key[node->pivot_dim];
    return findNode(node->children[child], key);
  }
}

void PFKDTree::clusterNode(PFKDTreeNode* node)
{
  int i;
  Key next_key;
  PFKDTreeNode* next_node;

  // Note: This code is tightly-coupled to a dimension_count of 3.
  for (i = 0; i < 3 * 3 * 3; i++)
  {
    next_key[0] = node->key[0] + (i / 9) - 1;
    next_key[1] = node->key[1] + ((i % 9) / 3) - 1;
    next_key[2] = node->key[2] + ((i % 9) % 3) - 1;

    if (node->key == next_key)
      continue;
    next_node = findNode(root_, next_key);
    if (next_node == NULL)
      continue;
    if (next_node->cluster >= 0)
    {
      ROS_ASSERT(next_node->cluster == node->cluster);
      continue;
    }
    next_node->cluster = node->cluster;
    clusterNode(next_node);
  }
}

PFKDTree::Key PFKDTree::getKey(const Eigen::Vector3d& pose)
{
  Key key;
  for (std::size_t idx = 0; idx < dimension_count; idx++)
  {
    key[idx] = std::floor(pose[idx] / cell_size_[idx]);
  }
  return key;
}

}  // namspace amcl
