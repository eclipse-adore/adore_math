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

#pragma once

#include <cmath>

#include <algorithm>
#include <array>

#include "adore_math/angles.h"

namespace adore::math
{

constexpr double PI     = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;

// Resolution and size
constexpr size_t TABLE_SIZE = 2048; // power of 2 for performance
constexpr double STEP       = TWO_PI / static_cast<double>( TABLE_SIZE );

// Generate sine table at compile time
template<size_t N>
constexpr std::array<double, N>
generate_sin_table()
{
  std::array<double, N> table{};
  for( size_t i = 0; i < N; ++i )
  {
    double angle = -PI + i * ( TWO_PI / N );
    table[i]     = std::sin( angle );
  }
  return table;
}

template<size_t N>
constexpr std::array<double, N>
generate_cos_table()
{
  std::array<double, N> table{};
  for( size_t i = 0; i < N; ++i )
  {
    double angle = -PI + i * ( TWO_PI / N );
    table[i]     = std::cos( angle );
  }
  return table;
}

// Tables (created at compile time)
constexpr auto sin_table = generate_sin_table<TABLE_SIZE>();
constexpr auto cos_table = generate_cos_table<TABLE_SIZE>();

// Fast sin/cos using lookup
inline double
fast_sin( double angle )
{
  angle              = normalize_angle( angle );
  const size_t index = static_cast<size_t>( ( angle + PI ) / STEP );
  return sin_table[std::min( index, TABLE_SIZE - 1 )];
}

inline double
fast_cos( double angle )
{
  angle              = normalize_angle( angle );
  const size_t index = static_cast<size_t>( ( angle + PI ) / STEP );
  return cos_table[std::min( index, TABLE_SIZE - 1 )];
}

} // namespace adore::math
