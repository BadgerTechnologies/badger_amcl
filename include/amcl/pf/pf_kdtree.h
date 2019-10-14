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
 * Desc: KD tree functions
 * Author: Andrew Howard
 * Date: 18 Dec 2002
 * CVS: $Id: pf_kdtree.h 6532 2008-06-11 02:45:56Z gbiggs $
 *************************************************************************/

#ifndef PF_KDTREE_H
#define PF_KDTREE_H

namespace amcl
{

// Info for a node in the tree
typedef struct pf_kdtree_node
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
  struct pf_kdtree_node *children[2];

} pf_kdtree_node_t;


// A kd tree
class KDTree
{
  public:
    // Create a tree
    KDTree(int max_size);

    // Destroy a tree
    ~KDTree();

    // Clear all entries from the tree
    void clear_kdtree();

    // Insert a pose into the tree
    void insert_pose(PFVector pose, double value);

    // Cluster the leaves in the tree
    void cluster();

    // Determine the probability estimate for the given pose
    double get_prob(PFVector pose);

    // Determine the cluster label for the given pose
    int get_cluster(PFVector pose);

    // Cell size
    double cell_size[3];

    // The root node of the tree
    pf_kdtree_node_t *root;

    // The number of nodes in the tree
    int node_count, node_max_count;
    pf_kdtree_node_t *nodes;

    // The number of leaf nodes in the tree
    int leaf_count;

    // Compare keys to see if they are equal
    bool equals(int key_a[], int key_b[]);

    // Insert a node into the tree
    pf_kdtree_node_t* insert_node(pf_kdtree_node_t *parent, pf_kdtree_node_t *node,
                                  int key[], double value);

    // Recursive node search
    pf_kdtree_node_t* find_node(pf_kdtree_node_t *node, int key[]);

    // Recursively label nodes in this cluster
    void cluster_node(pf_kdtree_node_t *node, int depth);
};

}

#endif
