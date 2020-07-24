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
  int key[3];
  key[0] = std::floor(pose[0] / cell_size_[0]);
  key[1] = std::floor(pose[1] / cell_size_[1]);
  key[2] = std::floor(pose[2] / cell_size_[2]);
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
  int key[3];
  PFKDTreeNode* node;
  key[0] = std::floor(pose[0] / cell_size_[0]);
  key[1] = std::floor(pose[1] / cell_size_[1]);
  key[2] = std::floor(pose[2] / cell_size_[2]);
  node = findNode(root_, key);
  if (node == NULL)
    return -1;
  return node->cluster;
}

int PFKDTree::getLeafCount()
{
  return leaf_count_;
}

PFKDTreeNode* PFKDTree::insertNode(PFKDTreeNode* node, int key[], double value, int depth)
{
  if (node == NULL)
  {
    node = makeLeafNode(key, value, depth);
  }
  else
  {
    if (equals(key, node->key))
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

PFKDTreeNode* PFKDTree::makeLeafNode(int key[], double value, int depth)
{
  nodes_.push_back(PFKDTreeNode());
  PFKDTreeNode* node = &nodes_.back();
  node->depth = depth;
  for (int i = 0; i < 3; i++)
    node->key[i] = key[i];
  node->value = value;
  node->pivot_dim = -1;
  node->cluster = -1;
  leaf_count_ += 1;
  return node;
}

void PFKDTree::traverseNode(PFKDTreeNode* node, int key[], double value, int depth)
{
  if (node->pivot_dim == -1)
  {
    int split, max_split = 0;
    for (int i = 0; i < 3; i++)
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

PFKDTreeNode* PFKDTree::findNode(PFKDTreeNode* node, int key[])
{
  if (node == NULL)
  {
    return NULL;
  }
  else if (equals(key, node->key))
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
  int next_key[3];
  PFKDTreeNode* next_node;

  for (i = 0; i < 3 * 3 * 3; i++)
  {
    next_key[0] = node->key[0] + (i / 9) - 1;
    next_key[1] = node->key[1] + ((i % 9) / 3) - 1;
    next_key[2] = node->key[2] + ((i % 9) % 3) - 1;

    if (equals(node->key, next_key))
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

bool PFKDTree::equals(int key_a[], int key_b[])
{
  return (key_a[0] == key_b[0]) and (key_a[1] == key_b[1]) and (key_a[2] == key_b[2]);
}

}  // namspace amcl
