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
 * Maintainter: Tyler Buchman (tyler_buchman@jabil.com)
 *************************************************************************/

#ifndef AMCL_PF_VECTOR_H
#define AMCL_PF_VECTOR_H

#include <stdio.h>

namespace amcl
{
  
// The basic vector
class PFVector
{
  public:
    double v[3];

    PFVector();

    // Check for NAN or INF in any component
    bool isFinite();

    // Simple vector addition
    static PFVector pfVectorAdd(PFVector a, PFVector b);

    // Simple vector subtraction
    static PFVector pfVectorSub(PFVector a, PFVector b);

    // Transform from local to global coords (a + b)
    static PFVector pfVectorCoordAdd(PFVector a, PFVector b);

    // Transform from global to local coords (a - b)
    static PFVector pfVectorCoordSub(PFVector a, PFVector b);
};


// The basic matrix
class PFMatrix
{
  public:
    double m[3][3];
    
    PFMatrix();

    // Check for NAN or INF in any component
    bool isFinite();

    // Decompose a covariance matrix [a] into a rotation matrix [r] and a
    // diagonal matrix [d] such that a = r * d * r^T.
    void decompose(PFMatrix *r, PFMatrix *d);
};

}

#endif
