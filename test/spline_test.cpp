/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/
#include "adore_math/spline.h"

#include <cmath>
#include <gtest/gtest.h>

#include <vector>

using tk::spline;

// -----------------------------------------------------------------------------
// Cubic spline: basic interpolation + derivatives for a quadratic
// -----------------------------------------------------------------------------

TEST( SplineTest, CsplineInterpolatesQuadraticAtKnots )
{
  std::vector<double> x{ 0.0, 1.0, 2.0, 3.0, 4.0 };
  std::vector<double> y;
  y.reserve( x.size() );
  for( double xi : x )
  {
    y.push_back( xi * xi ); // f(x) = x^2
  }

  spline s;
  // f''(x) = 2 for x^2
  s.set_boundary( spline::second_deriv, 2.0, spline::second_deriv, 2.0 );
  s.set_points( x, y, spline::cspline );

  const double eps_val  = 1e-10;
  const double eps_der1 = 1e-6;
  const double eps_der2 = 1e-5;

  // Spline must interpolate the data exactly at knots
  for( std::size_t i = 0; i < x.size(); ++i )
  {
    EXPECT_NEAR( s( x[i] ), y[i], eps_val ) << "Failed at knot x = " << x[i];
  }

  // First derivative should be close to analytical derivative 2x
  EXPECT_NEAR( s.deriv( 1, 0.0 ), 0.0, eps_der1 );
  EXPECT_NEAR( s.deriv( 1, 4.0 ), 8.0, eps_der1 );

  // Second derivative should be roughly constant ≈ 2 in the interior
  EXPECT_NEAR( s.deriv( 2, 2.0 ), 2.0, eps_der2 );
}

// -----------------------------------------------------------------------------
// Linear spline: piecewise linear interpolation and extrapolation
// -----------------------------------------------------------------------------

TEST( SplineTest, LinearInterpolationMatchesSegments )
{
  std::vector<double> x{ 0.0, 1.0, 2.0, 3.0 };
  std::vector<double> y{ 0.0, 1.0, 0.0, 1.0 };

  spline s;
  s.set_points( x, y, spline::linear );

  const double eps = 1e-12;

  // Midpoints between knots should match simple linear interpolation
  EXPECT_NEAR( s( 0.5 ), 0.5, eps ); // between (0,0) and (1,1)
  EXPECT_NEAR( s( 1.5 ), 0.5, eps ); // between (1,1) and (2,0)
  EXPECT_NEAR( s( 2.5 ), 0.5, eps ); // between (2,0) and (3,1)

  // Left extrapolation: slope of first segment is 1, y(0)=0 => f(-1) = -1
  EXPECT_NEAR( s( -1.0 ), -1.0, eps );

  // Right extrapolation: slope of last segment is 1, y(3)=1 => f(4) = 2
  EXPECT_NEAR( s( 4.0 ), 2.0, eps );
}

// -----------------------------------------------------------------------------
// Hermite spline: respects first-derivative boundary conditions
// -----------------------------------------------------------------------------

TEST( SplineTest, HermiteRespectsFirstDerivativeBoundary )
{
  // Perfectly linear data y = x
  std::vector<double> x{ 0.0, 1.0, 2.0 };
  std::vector<double> y{ 0.0, 1.0, 2.0 };

  spline s;
  // Enforce slope 1 at both ends
  s.set_boundary( spline::first_deriv, 1.0, spline::first_deriv, 1.0 );
  s.set_points( x, y, spline::cspline_hermite );

  const double eps_val  = 1e-8;
  const double eps_der1 = 1e-6;

  // Still interpolates the knots
  for( std::size_t i = 0; i < x.size(); ++i )
  {
    EXPECT_NEAR( s( x[i] ), y[i], eps_val ) << "Failed at knot x = " << x[i];
  }

  // Derivative at the boundaries should match the specified ones
  EXPECT_NEAR( s.deriv( 1, x.front() ), 1.0, eps_der1 );
  EXPECT_NEAR( s.deriv( 1, x.back() ), 1.0, eps_der1 );

  // The interior should stay close to the underlying line y = x
  EXPECT_NEAR( s( 0.5 ), 0.5, 1e-3 );
  EXPECT_NEAR( s( 1.5 ), 1.5, 1e-3 );
}

// -----------------------------------------------------------------------------
// make_monotonic: preserves values at knots and avoids overshoot between samples
// -----------------------------------------------------------------------------

TEST( SplineTest, MakeMonotonicPreservesInterpolationAndBounds )
{
  // Strictly increasing but with a slightly sharper jump
  std::vector<double> x{ 0.0, 1.0, 2.0, 3.0, 4.0 };
  std::vector<double> y{ 0.0, 0.1, 0.2, 0.5, 0.51 };

  spline s;
  s.set_points( x, y, spline::cspline );

  // Evaluate at knots before adjustment
  std::vector<double> before;
  before.reserve( x.size() );
  for( double xi : x )
  {
    before.push_back( s( xi ) );
  }

  (void) s.make_monotonic();

  // After make_monotonic, spline must still pass through all data points exactly
  const double eps_knot = 1e-12;
  for( std::size_t i = 0; i < x.size(); ++i )
  {
    EXPECT_NEAR( s( x[i] ), y[i], eps_knot ) << "Value at knot changed after make_monotonic, x = " << x[i];
  }

  // Additionally, sample between knots to ensure no overshoot beyond [y_i, y_{i+1}]
  for( std::size_t i = 0; i + 1 < x.size(); ++i )
  {
    const double xi   = x[i];
    const double xj   = x[i + 1];
    const double yi   = y[i];
    const double yj   = y[i + 1];
    const double ymin = std::min( yi, yj );
    const double ymax = std::max( yi, yj );

    for( int k = 1; k < 5; ++k )
    {
      const double xs = xi + ( xj - xi ) * ( static_cast<double>( k ) / 5.0 );
      const double ys = s( xs );

      EXPECT_GE( ys, ymin - 1e-6 ) << "Overshoot below segment bounds between x=" << xi << " and x=" << xj;
      EXPECT_LE( ys, ymax + 1e-6 ) << "Overshoot above segment bounds between x=" << xi << " and x=" << xj;
    }
  }
}

// -----------------------------------------------------------------------------
// band_matrix: basic solve for a small tridiagonal system
// -----------------------------------------------------------------------------

TEST( BandMatrixTest, SolvesTridiagonalSystem )
{
  // System:
  // [4 1 0] [x0]   [1]
  // [1 4 1] [x1] = [2]
  // [0 1 3] [x2]   [3]
  //
  // Solution (precomputed): x = [8/41, 9/41, 38/41]
  tk::internal::band_matrix A( 3, 1, 1 );

  A( 0, 0 ) = 4.0;
  A( 0, 1 ) = 1.0;
  A( 1, 0 ) = 1.0;
  A( 1, 1 ) = 4.0;
  A( 1, 2 ) = 1.0;
  A( 2, 1 ) = 1.0;
  A( 2, 2 ) = 3.0;

  std::vector<double> b{ 1.0, 2.0, 3.0 };

  auto x = A.lu_solve( b );

  const double eps = 1e-10;
  ASSERT_EQ( x.size(), 3u );
  EXPECT_NEAR( x[0], 8.0 / 41.0, eps );
  EXPECT_NEAR( x[1], 9.0 / 41.0, eps );
  EXPECT_NEAR( x[2], 38.0 / 41.0, eps );
}
