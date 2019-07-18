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
 * Date: 18 Dec 2002
 * CVS: $Id: pf_kdtree.c 7057 2008-10-02 00:44:06Z gbiggs $
 *************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include "pf_vector.h"
#include "pf_kdtree.h"

using namespace amcl;


///////////////////////////////////////////////////////////////////////////////
// Create a tree
KDTree::KDTree(int max_size)
{
  cell_size[0] = 0.50;
  cell_size[1] = 0.50;
  cell_size[2] = (10 * M_PI / 180);

  root = NULL;

  node_count = 0;
  node_max_count = max_size;
  nodes = (pf_kdtree_node_t*)calloc(node_max_count, sizeof(pf_kdtree_node_t));

  leaf_count = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destroy a tree
KDTree::~KDTree()
{
  free(nodes);
}

////////////////////////////////////////////////////////////////////////////////
// Clear all entries from the tree
void
KDTree::clear_kdtree()
{
  root = NULL;
  leaf_count = 0;
  node_count = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Insert a pose into the tree.
void
KDTree::insert_pose(PFVector pose, double value)
{
  int key[3];

  key[0] = floor(pose.v[0] / cell_size[0]);
  key[1] = floor(pose.v[1] / cell_size[1]);
  key[2] = floor(pose.v[2] / cell_size[2]);

  root = insert_node(NULL, root, key, value);
}

////////////////////////////////////////////////////////////////////////////////
// Determine the cluster label for the given pose
int
KDTree::get_cluster(PFVector pose)
{
  int key[3];
  pf_kdtree_node_t *node;

  key[0] = floor(pose.v[0] / cell_size[0]);
  key[1] = floor(pose.v[1] / cell_size[1]);
  key[2] = floor(pose.v[2] / cell_size[2]);

  node = find_node(root, key);
  if (node == NULL)
    return -1;
  return node->cluster;
}

////////////////////////////////////////////////////////////////////////////////
// Compare keys to see if they are equal
bool
KDTree::equals(int key_a[], int key_b[])
{
  return (key_a[0] == key_b[0]) and (key_a[1] == key_b[1]) and (key_a[2] == key_b[2]);
}

////////////////////////////////////////////////////////////////////////////////
// Insert a node into the tree
pf_kdtree_node_t*
KDTree::insert_node(pf_kdtree_node_t *parent,
                       pf_kdtree_node_t *node,
                       int key[], double value)
{
  int i;
  int split, max_split;

  // If the node doesnt exist yet...
  if (node == NULL)
  {
    assert(node_count < node_max_count);
    node = nodes + node_count++;
    memset(node, 0, sizeof(pf_kdtree_node_t));

    node->leaf = 1;

    if (parent == NULL)
      node->depth = 0;
    else
      node->depth = parent->depth + 1;

    for (i = 0; i < 3; i++)
      node->key[i] = key[i];

    node->value = value;
    leaf_count += 1;
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
      assert(node->pivot_dim >= 0);

      node->pivot_value = (key[node->pivot_dim] + node->key[node->pivot_dim]) / 2.0;

      if (key[node->pivot_dim] < node->pivot_value)
      {
        node->children[0] = insert_node(node, NULL, key, value);
        node->children[1] = insert_node(node, NULL, node->key, node->value);
      }
      else
      {
        node->children[0] = insert_node(node, NULL, node->key, node->value);
        node->children[1] = insert_node(node, NULL, key, value);
      }

      node->leaf = 0;
      leaf_count -= 1;
    }
  }

  // If the node exists, and it has children...
  else
  {
    assert(node->children[0] != NULL);
    assert(node->children[1] != NULL);

    if (key[node->pivot_dim] < node->pivot_value)
      insert_node(node, node->children[0], key, value);
    else
      insert_node(node, node->children[1], key, value);
  }

  return node;
}

////////////////////////////////////////////////////////////////////////////////
// Recursive node search
pf_kdtree_node_t*
KDTree::find_node(pf_kdtree_node_t *node, int key[])
{
  if (node->leaf)
  {
    //printf("find  : leaf %p %d %d %d\n", node, node->key[0], node->key[1], node->key[2]);

    // If the keys are the same...
    if (equals(key, node->key))
      return node;
    else
      return NULL;
  }
  else
  {
    assert(node->children[0] != NULL);
    assert(node->children[1] != NULL);

    // If the keys are different...
    if (key[node->pivot_dim] < node->pivot_value)
      return find_node(node->children[0], key);
    else
      return find_node(node->children[1], key);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Cluster the leaves in the tree
void
KDTree::cluster()
{
  int i;
  int queue_count, cluster_count;
  std::vector<pf_kdtree_node_t*> queue;
  pf_kdtree_node_t *node;

  queue_count = 0;

  // Put all the leaves in a queue
  for (i = 0; i < node_count; i++)
  {
    node = nodes + i;
    if (node->leaf)
    {
      node->cluster = -1;
      assert(queue_count < node_count);
      queue.push_back(node);
      queue_count++;

      // TESTING; remove
      assert(node == find_node(root, node->key));
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
    cluster_node(node, 0);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Recursively label nodes in this cluster
void
KDTree::cluster_node(pf_kdtree_node_t *node, int depth)
{
  int i;
  int nkey[3];
  pf_kdtree_node_t *nnode;

  for (i = 0; i < 3 * 3 * 3; i++)
  {
    nkey[0] = node->key[0] + (i / 9) - 1;
    nkey[1] = node->key[1] + ((i % 9) / 3) - 1;
    nkey[2] = node->key[2] + ((i % 9) % 3) - 1;

    nnode = find_node(root, nkey);
    if (nnode == NULL)
      continue;

    assert(nnode->leaf);

    // This node already has a label; skip it.  The label should be
    // consistent, however.
    if (nnode->cluster >= 0)
    {
      assert(nnode->cluster == node->cluster);
      continue;
    }

    // Label this node and recurse
    nnode->cluster = node->cluster;

    cluster_node(nnode, depth + 1);
  }
}
