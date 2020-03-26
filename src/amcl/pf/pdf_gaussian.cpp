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
 * Desc: Useful pdf functions
 * Author: Andrew Howard
 *************************************************************************/

#include "pf/pdf_gaussian.h"

#include <ros/console.h>

#include <cmath>

namespace amcl
{

/**************************************************************************
 * Gaussian
 *************************************************************************/

// Create a gaussian pdf
PDFGaussian::PDFGaussian(PFVector x, PFMatrix cx)
{
  PFMatrix m;

  x_ = x;
  cx_ = cx;

  // Decompose the convariance matrix into a rotation
  // matrix and a diagonal matrix.
  cx_.decompose(&cr_, &m);
  cd_.v[0] = std::sqrt(m.m[0][0]);
  cd_.v[1] = std::sqrt(m.m[1][1]);
  cd_.v[2] = std::sqrt(m.m[2][2]);
}

// Generate a sample from the the pdf.
PFVector PDFGaussian::sample()
{
  int i, j;
  PFVector r;
  PFVector v;

  // Generate a random vector
  for (i = 0; i < 3; i++)
  {
    r.v[i] = PDFGaussian::draw(cd_.v[i]);
  }

  for (i = 0; i < 3; i++)
  {
    v.v[i] = x_.v[i];
    for (j = 0; j < 3; j++)
      v.v[i] += cr_.m[i][j] * r.v[j];
  }
  return v;
}

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
double PDFGaussian::draw(double sigma)
{
  double x1, x2, w, r;

  do
  {
    do
    {
      r = drand48();
    } while (r == 0.0);
    x1 = 2.0 * r - 1.0;
    do
    {
      r = drand48();
    } while (r == 0.0);
    x2 = 2.0 * r - 1.0;
    w = x1 * x1 + x2 * x2;
  } while (w > 1.0 || w == 0.0);

  return (sigma * x2 * std::sqrt(-2.0 * std::log(w) / w));
}

}  // namespace amcl
