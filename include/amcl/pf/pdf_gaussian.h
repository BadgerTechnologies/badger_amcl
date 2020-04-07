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

#ifndef AMCL_PF_PDF_GAUSSIAN_H
#define AMCL_PF_PDF_GAUSSIAN_H

#include "pf/pf_vector.h"

namespace amcl
{

// Creates a gaussian probability density function (PDF) with a mean and covariance
// and a function to sample the PDF
class PDFGaussian
{
public:
  // Create a gaussian pdf
  PDFGaussian(PFVector x, PFMatrix cx);

  // Generate a sample from the the pdf.
  PFVector sample();

  // Draw randomly from a zero-mean Gaussian distribution, with standard
  // deviation sigma.
  // We use the polar form of the Box-Muller transformation, explained here:
  //   http://www.taygeta.com/random/gaussian.html
  static double draw(double sigma);

private:
  // Mean, covariance and inverse covariance
  PFVector x_;
  PFMatrix cx_;

  // Decomposed covariance matrix (rotation * diagonal)
  PFMatrix cr_;
  PFVector cd_;
};

}  // namespace amcl

#endif  // AMCL_PF_PDF_GAUSSIAN_H
