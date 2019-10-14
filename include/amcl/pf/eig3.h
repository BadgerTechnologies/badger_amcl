
/* Eigen-decomposition for symmetric 3x3 real matrices.
   Public domain, copied from the public domain Java library JAMA. */

#ifndef _eig_h

/* Symmetric matrix A => eigenvectors in columns of V, corresponding
   eigenvalues in d. */

#include <vector>

namespace amcl
{

class eig3
{

public:
  static void eigen_decomposition(std::vector<std::vector<double>> A,
                                  std::vector<std::vector<double>> V,
                                  std::vector<double> d);
private:
  static const int n = 3;
  static double hypot2(double x, double y);
  static void tred2(std::vector<std::vector<double>> V, std::vector<double> d, std::vector<double> e);
  static void tql2(std::vector<std::vector<double>> V, std::vector<double> d, std::vector<double> e);
};

}

#endif
