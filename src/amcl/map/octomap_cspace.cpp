/*
 *
 *       AMCL Octomap Class
 *        by Tyler Buchman
 *        2019
 *
 */

#include <queue>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeDistance.h>
#include <octomap/OcTreeKey.h>
#include <octomap/OcTreeDataNode.h>
#include "map.h"
#include "octomap.h"

#include "ros/ros.h"

using namespace amcl;

bool
OctoMap::enqueue(int i, int j, int k, int src_i, int src_j, int src_k,
                 std::priority_queue<CellData>& Q)
{
  int di = abs(i - src_i);
  int dj = abs(j - src_j);
  int dk = abs(k - src_k);
  double distance = cdm->distances_[di][dj][dk];

  if(distance > cdm->cell_radius_)
    return false;

  setOccDist(i, j, k, distance * scale);

  CellData cell = CellData(this);
  cell.i_ = i;
  cell.j_ = j;
  cell.k_ = k;
  cell.src_i_ = src_i;
  cell.src_j_ = src_j;
  cell.src_k_ = src_k;

  Q.push(cell);

  return true;
}

void
OctoMap::updateCSpace(double _max_occ_dist)
{
  max_occ_dist = _max_occ_dist;
  updateCSpace();
}

// Update the cspace distance values
void
OctoMap::updateCSpace()
{
  if(distances)
  {
    distances->clear();
  }
  std::priority_queue<CellData> Q;
  octomap::OcTree* marked = new octomap::OcTree(scale);

  distances = new octomap::OcTreeDistance(scale, max_occ_dist);

  if(max_occ_dist == 0.0)
  {
    return;
  }

  if(!cdm || (cdm->scale_ != scale) || (std::fabs(cdm->max_dist_ - max_occ_dist) > EPSILON_DOUBLE))
  {
    if(cdm)
    {
      ROS_DEBUG("Deleting existing cdm");
      delete cdm;
    }
    cdm = new CachedDistanceMap(scale, max_occ_dist);
  }

  // Enqueue all the obstacle cells
  CellData cell = CellData(this);
  for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
      end=octree->end_leafs(); it != end; ++it)
  {
    if(octree->isNodeOccupied(*it))
    {
      std::vector<double> world_coords;
      std::vector<int> map_coords;
      int i, j, k;
      world_coords.push_back(it.getX());
      world_coords.push_back(it.getY());
      world_coords.push_back(it.getZ());
      map_coords = convertWorldToMap(world_coords);
      i = map_coords[0];
      j = map_coords[1];
      k = map_coords[2];
      setOccDist(i, j, k, 0.0);
      cell.src_i_ = cell.i_ = i;
      cell.src_j_ = cell.j_ = j;
      cell.src_k_ = cell.k_ = k;
      marked->updateNode(i, j, k, true);
      Q.push(cell);
    }
    else
    {
      octomap::OcTreeKey key = it.getIndexKey();
      int i = key[0], j = key[1], k = key[2];
      setOccDist(i, j, k, max_occ_dist);
    }
  }

  double occThresh = octree->getOccupancyThres();

  while(!Q.empty())
  {
    CellData current_cell = Q.top();
    if(current_cell.i_ > 0)
    {
      int i = current_cell.i_-1, j = current_cell.j_, k = current_cell.k_;
      octomap::OcTreeKey key(i, j, k);
      octomap::OcTreeNode* node = marked->search(key);
      if(node == nullptr or not node->getOccupancy() > occThresh)
        marked->updateNode(key, enqueue(i, j, k, current_cell.src_i_, current_cell.src_j_, current_cell.src_k_, Q));
    }
    if(current_cell.j_ > 0)
    {
      int i = current_cell.i_, j = current_cell.j_-1, k = current_cell.k_;
      octomap::OcTreeKey key(i, j, k);
      octomap::OcTreeNode* node = marked->search(key);
      if(node == nullptr or not node->getOccupancy() > occThresh)
        marked->updateNode(key, enqueue(i, j, k, current_cell.src_i_, current_cell.src_j_, current_cell.src_k_, Q));
    }
    if(current_cell.k_ > 0)
    {
      int i = current_cell.i_, j = current_cell.j_, k = current_cell.k_-1;
      octomap::OcTreeKey key(i, j, k);
      octomap::OcTreeNode* node = marked->search(key);
      if(node == nullptr or not node->getOccupancy() > occThresh)
        marked->updateNode(key, enqueue(i, j, k, current_cell.src_i_, current_cell.src_j_, current_cell.src_k_, Q));
    }
    if((int)current_cell.i_ < cells_x - 1)
    {
      int i = current_cell.i_+1, j = current_cell.j_, k = current_cell.k_;
      octomap::OcTreeKey key(i, j, k);
      octomap::OcTreeNode* node = marked->search(key);
      if(node == nullptr or not node->getOccupancy() > occThresh)
        marked->updateNode(key, enqueue(i, j, k, current_cell.src_i_, current_cell.src_j_, current_cell.src_k_, Q));
    }
    if((int)current_cell.j_ < cells_y - 1)
    {
      int i = current_cell.i_, j = current_cell.j_+1, k = current_cell.k_;
      octomap::OcTreeKey key(i, j, k);
      octomap::OcTreeNode* node = marked->search(key);
      if(node == nullptr or not node->getOccupancy() > occThresh)
        marked->updateNode(key, enqueue(i, j, k, current_cell.src_i_, current_cell.src_j_, current_cell.src_k_, Q));
    }
    if((int)current_cell.k_ < cells_z - 1)
    {
      int i = current_cell.i_, j = current_cell.j_, k = current_cell.k_+1;
      octomap::OcTreeKey key(i, j, k);
      octomap::OcTreeNode* node = marked->search(key);
      if(node == nullptr or not node->getOccupancy() > occThresh)
        marked->updateNode(key, enqueue(i, j, k, current_cell.src_i_, current_cell.src_j_, current_cell.src_k_, Q));
    }
    Q.pop();
  }
}

double
OctoMap::getOccDist(int i, int j, int k, bool pub)
{
  double val = getOccDist(i, j, k);
  if(pub)
  {
    ROS_INFO("i: %d, j: %d, k: %d, dist: %f", i, j, k, val);
  }
  return val;
}

double
OctoMap::getOccDist(int i, int j, int k)
{
  octomap::OcTreeKey key(i, j, k);
  octomap::OcTreeDataNode<double>* node = distances->search(key);
  if(node and node->getValue() < max_occ_dist)
  {
    return node->getValue();
  }
  return 1*max_occ_dist;
}

double
OctoMap::getOccDist(int i, int j)
{
  double distance = max_occ_dist;
  for(int k = 0; k < lidar_height / scale; k++)
  {
    distance = std::min(distance, getOccDist(i, j, k));
  }
  return distance;
}

void
OctoMap::setOccDist(int i, int j, int k, double d)
{
  int index = floor(d*100);
  octomap::OcTreeKey key(i, j, k);
  distances->setDistanceForKey(key, d);
}
