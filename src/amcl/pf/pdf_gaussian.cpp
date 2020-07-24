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

#include "pf/pdf_gaussian.h"

#include <stdlib.h>

#include <cmath>

#include <Eigen/Eigenvalues>

namespace badger_amcl
{

PDFGaussian::PDFGaussian(const Eigen::Vector3d& x, const Eigen::Matrix3d& cx)
{
  Eigen::Matrix3d m;

  x_ = x;
  cx_ = cx;

  // Decompose the convariance matrix into a rotation
  // matrix and a diagonal matrix.
  decompose(cx_, &cr_, &m);
  cd_[0] = std::sqrt(m(0, 0));
  cd_[1] = std::sqrt(m(1, 1));
  cd_[2] = std::sqrt(m(2, 2));
}

PDFGaussian::PDFGaussian(const Eigen::Vector3d& x, const Eigen::Matrix3d& cx, int seed) : PDFGaussian(x, cx)
{
  srand48(seed);
}

// Generate a sample from the the pdf.
void PDFGaussian::sample(Eigen::Vector3d* v)
{
  int i, j;
  Eigen::Vector3d r;

  // Generate a random vector
  for (i = 0; i < 3; i++)
  {
    r[i] = PDFGaussian::draw(cd_[i]);
  }

  for (i = 0; i < 3; i++)
  {
    (*v)(i) = x_[i];
    for (j = 0; j < 3; j++)
      (*v)(i) += cr_(i, j) * r[j];
  }
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

void PDFGaussian::decompose(const Eigen::Matrix3d& m, Eigen::Matrix3d* r, Eigen::Matrix3d* d)
{
  int i, j;

  Eigen::Matrix3d aa;
  Eigen::Vector3cd eval;
  Eigen::Matrix3cd evec;

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      aa(i, j) = m(i, j);
    }
  }

  Eigen::EigenSolver<Eigen::MatrixXd> solver(aa, true);
  eval = solver.eigenvalues();
  evec = solver.eigenvectors();

  for (i = 0; i < 3; i++)
  {
    (*d)(i, i) = eval[i].real();
    for (j = 0; j < 3; j++)
    {
      (*r)(i, j) = evec(i, j).real();
    }
  }
}

}  // namespace amcl
