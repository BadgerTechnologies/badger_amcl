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
 * Desc: Vector functions
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf_vector.c 6345 2008-04-17 01:36:39Z gerkey $
 *************************************************************************/

#include <math.h>
#include <vector>

#include "pf_vector.h"
#include "eig3.h"

using namespace amcl;

// Return a zero vector
PFVector::PFVector()
{
  v[0] = 0.0;
  v[1] = 0.0;
  v[2] = 0.0;
}

// Check for NAN or INF in any component
bool
PFVector::is_finite()
{
  int i;
  
  for (i = 0; i < 3; i++)
    if (!finite(v[i]))
      return 0;
  return 1;
}

// Simple vector addition
PFVector
PFVector::pf_vector_add(PFVector a, PFVector b)
{
  PFVector c;

  c.v[0] = a.v[0] + b.v[0];
  c.v[1] = a.v[1] + b.v[1];
  c.v[2] = a.v[2] + b.v[2];
  
  return c;
}


// Simple vector subtraction
PFVector
PFVector::pf_vector_sub(PFVector a, PFVector b)
{
  PFVector c;

  c.v[0] = a.v[0] - b.v[0];
  c.v[1] = a.v[1] - b.v[1];
  c.v[2] = a.v[2] - b.v[2];
  
  return c;
}


// Transform from local to global coords (a + b)
PFVector
PFVector::pf_vector_coord_add(PFVector a, PFVector b)
{
  PFVector c;

  c.v[0] = b.v[0] + a.v[0] * cos(b.v[2]) - a.v[1] * sin(b.v[2]);
  c.v[1] = b.v[1] + a.v[0] * sin(b.v[2]) + a.v[1] * cos(b.v[2]);
  c.v[2] = b.v[2] + a.v[2];
  c.v[2] = atan2(sin(c.v[2]), cos(c.v[2]));
  
  return c;
}


// Transform from global to local coords (a - b)
PFVector
PFVector::pf_vector_coord_sub(PFVector a, PFVector b)
{
  PFVector c;

  c.v[0] = +(a.v[0] - b.v[0]) * cos(b.v[2]) + (a.v[1] - b.v[1]) * sin(b.v[2]);
  c.v[1] = -(a.v[0] - b.v[0]) * sin(b.v[2]) + (a.v[1] - b.v[1]) * cos(b.v[2]);
  c.v[2] = a.v[2] - b.v[2];
  c.v[2] = atan2(sin(c.v[2]), cos(c.v[2]));
  
  return c;
}


// Return a zero matrix
PFMatrix::PFMatrix()
{
  int i, j;

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      m[i][j] = 0.0;
}

// Check for NAN or INF in any component
bool
PFMatrix::is_finite()
{
  int i, j;
  
  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      if (!finite(m[i][j]))
        return false; 
  return true;
}

// Decompose a covariance matrix [a] into a rotation matrix [r] and a diagonal
// matrix [d] such that a = r d r^T.
void
PFMatrix::decompose(PFMatrix *r, PFMatrix *d)
{
  int i, j;

  std::vector<std::vector<double>> aa(3, std::vector<double>(3, 0.0));
  std::vector<double> eval(3, 0.0);
  std::vector<std::vector<double>> evec(3, std::vector<double>(3, 0.0));

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      aa[i][j] = m[i][j];
    }
  }

  eig3::eigen_decomposition(aa, evec, eval);

  *d = PFMatrix();
  for (i = 0; i < 3; i++)
  {
    d->m[i][i] = eval[i];
    for (j = 0; j < 3; j++)
    {
      r->m[i][j] = evec[i][j];
    }
  }
}

