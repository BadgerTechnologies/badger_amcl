/*
 *  Player - One Hell of a Robot Server
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
/**************************************************************************
 * Desc: kd-tree functions
 * Author: Andrew Howard
 * Maintainer: Tyler Buchman (tyler_buchman@jabil.com)
 *************************************************************************/

#include "pf/pf_kdtree.h"

#include <math.h>
#include <ros/assert.h>
#include <stdlib.h>

#include <string>
#include <vector>

using namespace amcl;

///////////////////////////////////////////////////////////////////////////////
// Create a tree
PFKDTree::PFKDTree(int max_size)
{
  cell_size_[0] = 0.50;
  cell_size_[1] = 0.50;
  cell_size_[2] = (10 * M_PI / 180);

  root_ = NULL;

  node_count_ = 0;
  node_max_count_ = max_size;
  nodes_ = (PFKDTreeNode*)calloc(node_max_count_, sizeof(PFKDTreeNode));

  leaf_count_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destroy a tree
PFKDTree::~PFKDTree()
{
  free(nodes_);
}

////////////////////////////////////////////////////////////////////////////////
// Clear all entries from the tree
void PFKDTree::clearKDTree()
{
  root_ = NULL;
  leaf_count_ = 0;
  node_count_ = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Insert a pose into the tree.
void PFKDTree::insertPose(PFVector pose, double value)
{
  int key[3];

  key[0] = floor(pose.v[0] / cell_size_[0]);
  key[1] = floor(pose.v[1] / cell_size_[1]);
  key[2] = floor(pose.v[2] / cell_size_[2]);

  root_ = insertNode(NULL, root_, key, value);
}

////////////////////////////////////////////////////////////////////////////////
// Determine the cluster label for the given pose
int PFKDTree::getCluster(PFVector pose)
{
  int key[3];
  PFKDTreeNode* node;

  key[0] = floor(pose.v[0] / cell_size_[0]);
  key[1] = floor(pose.v[1] / cell_size_[1]);
  key[2] = floor(pose.v[2] / cell_size_[2]);

  node = findNode(root_, key);
  if (node == NULL)
    return -1;
  return node->cluster;
}

////////////////////////////////////////////////////////////////////////////////
// Compare keys to see if they are equal
bool PFKDTree::equals(int key_a[], int key_b[])
{
  return (key_a[0] == key_b[0]) and (key_a[1] == key_b[1]) and (key_a[2] == key_b[2]);
}

////////////////////////////////////////////////////////////////////////////////
// Insert a node into the tree
PFKDTreeNode* PFKDTree::insertNode(PFKDTreeNode* parent, PFKDTreeNode* node, int key[], double value)
{
  int i;
  int split, max_split;

  // If the node doesnt exist yet...
  if (node == NULL)
  {
    ROS_ASSERT(node_count_ < node_max_count_);
    node = nodes_ + node_count_++;
    memset(node, 0, sizeof(PFKDTreeNode));

    node->leaf = 1;

    if (parent == NULL)
      node->depth = 0;
    else
      node->depth = parent->depth + 1;

    for (i = 0; i < 3; i++)
      node->key[i] = key[i];

    node->value = value;
    leaf_count_ += 1;
  }

  // If the node exists, and it is a leaf node...
  else if (node->leaf)
  {
    // If the keys are equal, increment the value
    if (equals(key, node->key))
    {
      node->value += value;
    }

    // The keys are not equal, so split this node
    else
    {
      // Find the dimension with the largest variance and do a mean
      // split
      max_split = 0;
      node->pivot_dim = -1;
      for (i = 0; i < 3; i++)
      {
        split = abs(key[i] - node->key[i]);
        if (split > max_split)
        {
          max_split = split;
          node->pivot_dim = i;
        }
      }
      ROS_ASSERT(node->pivot_dim >= 0);

      node->pivot_value = (key[node->pivot_dim] + node->key[node->pivot_dim]) / 2.0;

      if (key[node->pivot_dim] < node->pivot_value)
      {
        node->children[0] = insertNode(node, NULL, key, value);
        node->children[1] = insertNode(node, NULL, node->key, node->value);
      }
      else
      {
        node->children[0] = insertNode(node, NULL, node->key, node->value);
        node->children[1] = insertNode(node, NULL, key, value);
      }

      node->leaf = 0;
      leaf_count_ -= 1;
    }
  }

  // If the node exists, and it has children...
  else
  {
    ROS_ASSERT(node->children[0] != NULL);
    ROS_ASSERT(node->children[1] != NULL);

    if (key[node->pivot_dim] < node->pivot_value)
      insertNode(node, node->children[0], key, value);
    else
      insertNode(node, node->children[1], key, value);
  }

  return node;
}

////////////////////////////////////////////////////////////////////////////////
// Recursive node search
PFKDTreeNode* PFKDTree::findNode(PFKDTreeNode* node, int key[])
{
  if (node->leaf)
  {
    // printf("find  : leaf %p %d %d %d\n", node, node->key[0], node->key[1], node->key[2]);

    // If the keys are the same...
    if (equals(key, node->key))
      return node;
    else
      return NULL;
  }
  else
  {
    ROS_ASSERT(node->children[0] != NULL);
    ROS_ASSERT(node->children[1] != NULL);

    // If the keys are different...
    if (key[node->pivot_dim] < node->pivot_value)
      return findNode(node->children[0], key);
    else
      return findNode(node->children[1], key);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Cluster the leaves in the tree
void PFKDTree::cluster()
{
  int i;
  int queue_count, cluster_count;
  std::vector<PFKDTreeNode*> queue;
  PFKDTreeNode* node;

  queue_count = 0;

  // Put all the leaves in a queue
  for (i = 0; i < node_count_; i++)
  {
    node = nodes_ + i;
    if (node->leaf)
    {
      node->cluster = -1;
      ROS_ASSERT(queue_count < node_count_);
      queue.push_back(node);
      queue_count++;

      // TESTING; remove
      ROS_ASSERT(node == findNode(root_, node->key));
    }
  }

  cluster_count = 0;

  // Do connected components for each node
  while (queue_count > 0)
  {
    node = queue[--queue_count];

    // If this node has already been labelled, skip it
    if (node->cluster >= 0)
      continue;

    // Assign a label to this cluster
    node->cluster = cluster_count++;

    // Recursively label nodes in this cluster
    clusterNode(node, 0);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Recursively label nodes in this cluster
void PFKDTree::clusterNode(PFKDTreeNode* node, int depth)
{
  int i;
  int nkey[3];
  PFKDTreeNode* nnode;

  for (i = 0; i < 3 * 3 * 3; i++)
  {
    nkey[0] = node->key[0] + (i / 9) - 1;
    nkey[1] = node->key[1] + ((i % 9) / 3) - 1;
    nkey[2] = node->key[2] + ((i % 9) % 3) - 1;

    nnode = findNode(root_, nkey);
    if (nnode == NULL)
      continue;

    ROS_ASSERT(nnode->leaf);

    // This node already has a label; skip it.  The label should be
    // consistent, however.
    if (nnode->cluster >= 0)
    {
      ROS_ASSERT(nnode->cluster == node->cluster);
      continue;
    }

    // Label this node and recurse
    nnode->cluster = node->cluster;

    clusterNode(nnode, depth + 1);
  }
}

int PFKDTree::getLeafCount()
{
  return leaf_count_;
}
