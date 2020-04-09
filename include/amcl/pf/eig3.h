/* Eigen-decomposition for symmetric 3x3 real matrices.
   Public domain, copied from the public domain Java library JAMA. */

#ifndef AMCL_PF_EIG3_H
#define AMCL_PF_EIG3_H

/* Symmetric matrix A => eigenvectors in columns of V, corresponding
   eigenvalues in d. */

#include "pf/pf_vector.h"

namespace amcl
{
class EIG3
{
public:
  static void eigenDecomposition(const PFMatrix& A, PFMatrix* V, PFVector* d);

private:
  static constexpr int N = 3;
  static void tred2(PFMatrix* V, PFVector* d, PFVector* e);
  static void tql2(PFMatrix* V, PFVector* d, PFVector* e);
};

}  // namespace amcl

#endif  // AMCL_PF_EIG3_H
