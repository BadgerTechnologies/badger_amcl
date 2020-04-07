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
/**************************************************************************
 * Desc: Vector functions
 * Author: Andrew Howard
 *************************************************************************/

#include "pf/pf_vector.h"

#include <cmath>
#include <vector>

#include "pf/eig3.h"

namespace amcl
{

PFVector::PFVector()
{
  v[0] = 0.0;
  v[1] = 0.0;
  v[2] = 0.0;
}

// Check for NAN or INF in any component
bool PFVector::isFinite()
{
  int i;
  for (i = 0; i < 3; i++)
    if (!std::isfinite(v[i]))
      return false;
  return true;
}

// Simple vector addition
PFVector PFVector::pfVectorAdd(PFVector a, PFVector b)
{
  PFVector c;
  c.v[0] = a.v[0] + b.v[0];
  c.v[1] = a.v[1] + b.v[1];
  c.v[2] = a.v[2] + b.v[2];
  return c;
}

// Simple vector subtraction
PFVector PFVector::pfVectorSub(PFVector a, PFVector b)
{
  PFVector c;
  c.v[0] = a.v[0] - b.v[0];
  c.v[1] = a.v[1] - b.v[1];
  c.v[2] = a.v[2] - b.v[2];
  return c;
}

// Transform from local to global coords (a + b)
PFVector PFVector::pfVectorCoordAdd(PFVector a, PFVector b)
{
  PFVector c;
  c.v[0] = b.v[0] + a.v[0] * std::cos(b.v[2]) - a.v[1] * std::sin(b.v[2]);
  c.v[1] = b.v[1] + a.v[0] * std::sin(b.v[2]) + a.v[1] * std::cos(b.v[2]);
  c.v[2] = b.v[2] + a.v[2];
  c.v[2] = std::atan2(std::sin(c.v[2]), std::cos(c.v[2]));
  return c;
}

// Transform from global to local coords (a - b)
PFVector PFVector::pfVectorCoordSub(PFVector a, PFVector b)
{
  PFVector c;
  c.v[0] = +(a.v[0] - b.v[0]) * std::cos(b.v[2]) + (a.v[1] - b.v[1]) * std::sin(b.v[2]);
  c.v[1] = -(a.v[0] - b.v[0]) * std::sin(b.v[2]) + (a.v[1] - b.v[1]) * std::cos(b.v[2]);
  c.v[2] = a.v[2] - b.v[2];
  c.v[2] = std::atan2(std::sin(c.v[2]), std::cos(c.v[2]));
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
bool PFMatrix::isFinite()
{
  int i, j;

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      if (!std::isfinite(m[i][j]))
        return false;
  return true;
}

// Decompose a covariance matrix [a] into a rotation matrix [r] and a diagonal
// matrix [d] such that a = r d r^T.
void PFMatrix::decompose(PFMatrix* r, PFMatrix* d)
{
  int i, j;

  PFMatrix aa;
  PFVector eval;
  PFMatrix evec;

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      aa.m[i][j] = m[i][j];
    }
  }

  EIG3::eigenDecomposition(aa, &evec, &eval);

  *d = PFMatrix();
  for (i = 0; i < 3; i++)
  {
    d->m[i][i] = eval.v[i];
    for (j = 0; j < 3; j++)
    {
      r->m[i][j] = evec.m[i][j];
    }
  }
}

}  // namspace amcl
