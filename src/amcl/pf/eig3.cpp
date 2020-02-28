/**************************************************************************
 * Desc: Eigen-decomposition functions
 * Maintainter: Tyler Buchman (tyler_buchman@jabil.com)
 *************************************************************************/

/* Eigen decomposition code for symmetric 3x3 matrices, copied from the public
   domain Java Matrix library JAMA. */

#include "pf/eig3.h"

#include <math.h>
#include <ros/console.h>

#include <algorithm>

using namespace amcl;

double EIG3::hypot2(double x, double y)
{
  return sqrt(x * x + y * y);
}

// Symmetric Householder reduction to tridiagonal form.

void EIG3::tred2(PFMatrix& V, PFVector* d, PFVector* e)
{
  //  This is derived from the Algol procedures tred2 by
  //  Bowdler, Martin, Reinsch, and Wilkinson, Handbook for
  //  Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
  //  Fortran subroutine in EISPACK.

  int i, j, k;
  double f, g, h, hh;
  for (j = 0; j < N; j++)
  {
    d->v[j] = V.m[N - 1][j];
  }

  // Householder reduction to tridiagonal form.

  for (i = N - 1; i > 0; i--)
  {
    // Scale to avoid under/overflow.

    double scale = 0.0;
    double h = 0.0;
    for (k = 0; k < i; k++)
    {
      scale = scale + fabs(d->v[k]);
    }
    if (scale == 0.0)
    {
      e->v[i] = d->v[i - 1];
      for (j = 0; j < i; j++)
      {
        d->v[j] = V.m[i - 1][j];
        V.m[i][j] = 0.0;
        V.m[j][i] = 0.0;
      }
    }
    else
    {
      // Generate Householder vector.

      for (k = 0; k < i; k++)
      {
        d->v[k] /= scale;
        h += d->v[k] * d->v[k];
      }
      f = d->v[i - 1];
      g = sqrt(h);
      if (f > 0)
      {
        g = -g;
      }
      e->v[i] = scale * g;
      h = h - f * g;
      d->v[i - 1] = f - g;
      for (j = 0; j < i; j++)
      {
        e->v[j] = 0.0;
      }

      // Apply similarity transformation to remaining columns.

      for (j = 0; j < i; j++)
      {
        f = d->v[j];
        V.m[j][i] = f;
        g = e->v[j] + V.m[j][j] * f;
        for (k = j + 1; k <= i - 1; k++)
        {
          g += V.m[k][j] * d->v[k];
          e->v[k] += V.m[k][j] * f;
        }
        e->v[j] = g;
      }
      f = 0.0;
      for (j = 0; j < i; j++)
      {
        e->v[j] /= h;
        f += e->v[j] * d->v[j];
      }
      hh = f / (h + h);
      for (j = 0; j < i; j++)
      {
        e->v[j] -= hh * d->v[j];
      }
      for (j = 0; j < i; j++)
      {
        f = d->v[j];
        g = e->v[j];
        for (k = j; k <= i - 1; k++)
        {
          V.m[k][j] -= (f * e->v[k] + g * d->v[k]);
        }
        d->v[j] = V.m[i - 1][j];
        V.m[i][j] = 0.0;
      }
    }
    d->v[i] = h;
  }

  // Accumulate transformations.

  for (i = 0; i < N - 1; i++)
  {
    V.m[N - 1][i] = V.m[i][i];
    V.m[i][i] = 1.0;
    h = d->v[i + 1];
    if (h != 0.0)
    {
      for (k = 0; k <= i; k++)
      {
        d->v[k] = V.m[k][i + 1] / h;
      }
      for (j = 0; j <= i; j++)
      {
        g = 0.0;
        for (k = 0; k <= i; k++)
        {
          g += V.m[k][i + 1] * V.m[k][j];
        }
        for (k = 0; k <= i; k++)
        {
          V.m[k][j] -= g * d->v[k];
        }
      }
    }
    for (k = 0; k <= i; k++)
    {
      V.m[k][i + 1] = 0.0;
    }
  }
  for (j = 0; j < N; j++)
  {
    d->v[j] = V.m[N - 1][j];
    V.m[N - 1][j] = 0.0;
  }
  V.m[N - 1][N - 1] = 1.0;
  e->v[0] = 0.0;
}

// Symmetric tridiagonal QL algorithm.

void EIG3::tql2(PFMatrix& V, PFVector& d, PFVector& e)
{
  //  This is derived from the Algol procedures tql2, by
  //  Bowdler, Martin, Reinsch, and Wilkinson, Handbook for
  //  Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
  //  Fortran subroutine in EISPACK.

  int i, j, m, l, k;
  double g, p, r, dl1, h, f, tst1, eps;
  double c, c2, c3, el1, s, s2;

  for (i = 1; i < N; i++)
  {
    e.v[i - 1] = e.v[i];
  }
  e.v[N - 1] = 0.0;

  f = 0.0;
  tst1 = 0.0;
  eps = pow(2.0, -52.0);
  for (l = 0; l < N; l++)
  {
    // Find small subdiagonal element

    tst1 = std::max(fabs(d.v[l]) + fabs(e.v[l]), tst1);
    m = l;
    while (m < N)
    {
      if (fabs(e.v[m]) <= eps * tst1)
      {
        break;
      }
      m++;
    }

    // If m == l, d[l] is an eigenvalue,
    // otherwise, iterate.

    if (m > l)
    {
      int iter = 0;
      do
      {
        iter = iter + 1;  // (Could check iteration count here.)

        // Compute implicit shift

        g = d.v[l];
        p = (d.v[l + 1] - g) / (2.0 * e.v[l]);
        r = hypot2(p, 1.0);
        if (p < 0)
        {
          r = -r;
        }
        d.v[l] = e.v[l] / (p + r);
        d.v[l + 1] = e.v[l] * (p + r);
        dl1 = d.v[l + 1];
        h = g - d.v[l];
        for (i = l + 2; i < N; i++)
        {
          d.v[i] -= h;
        }
        f = f + h;

        // Implicit QL transformation.

        p = d.v[m];
        c = 1.0;
        c2 = c;
        c3 = c;
        el1 = e.v[l + 1];
        s = 0.0;
        s2 = 0.0;
        for (i = m - 1; i >= l; i--)
        {
          c3 = c2;
          c2 = c;
          s2 = s;
          g = c * e.v[i];
          h = c * p;
          r = hypot2(p, e.v[i]);
          e.v[i + 1] = s * r;
          s = e.v[i] / r;
          c = p / r;
          p = c * d.v[i] - s * g;
          d.v[i + 1] = h + s * (c * g + s * d.v[i]);

          // Accumulate transformation.

          for (k = 0; k < N; k++)
          {
            h = V.m[k][i + 1];
            V.m[k][i + 1] = s * V.m[k][i] + c * h;
            V.m[k][i] = c * V.m[k][i] - s * h;
          }
        }
        p = -s * s2 * c3 * el1 * e.v[l] / dl1;
        e.v[l] = s * p;
        d.v[l] = c * p;

        // Check for convergence.

      } while (fabs(e.v[l]) > eps * tst1);
    }
    d.v[l] = d.v[l] + f;
    e.v[l] = 0.0;
  }

  // Sort eigenvalues and corresponding vectors.

  for (i = 0; i < N - 1; i++)
  {
    k = i;
    p = d.v[i];
    for (j = i + 1; j < N; j++)
    {
      if (d.v[j] < p)
      {
        k = j;
        p = d.v[j];
      }
    }
    if (k != i)
    {
      d.v[k] = d.v[i];
      d.v[i] = p;
      for (j = 0; j < N; j++)
      {
        p = V.m[j][i];
        V.m[j][i] = V.m[j][k];
        V.m[j][k] = p;
      }
    }
  }
}

void EIG3::eigenDecomposition(const PFMatrix& A, PFMatrix* V, PFVector* d)
{
  int i, j;
  PFVector e;
  for (i = 0; i < N; i++)
  {
    for (j = 0; j < N; j++)
    {
      V->m[i][j] = A.m[i][j];
    }
  }
  EIG3::tred2(*V, d, &e);
  EIG3::tql2(*V, *d, e);
}
