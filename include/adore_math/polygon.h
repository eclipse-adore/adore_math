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
#include <vector>

#include "adore_math/point.h"

namespace adore

{
namespace math
{
struct Polygon2d
{
  std::vector<Point2d> points;

  Polygon2d() = default;

  // Construct from a vector of "point-like" objects (must have .x and .y)
  template<typename PointLike>
  explicit Polygon2d( const std::vector<PointLike>& input_points )
  {
    points.reserve( input_points.size() );
    for( const auto& p : input_points )
    {
      points.push_back( Point2d{ p.x, p.y } );
    }
  }

  // Construct from a flat vector of doubles: [x0, y0, x1, y1, ...]
  explicit Polygon2d( const std::vector<double>& coords )
  {
    // If odd sized, last coordinate is ignored
    const std::size_t pair_count = coords.size() / 2;
    points.reserve( pair_count );

    for( std::size_t i = 0; i < pair_count; ++i )
    {
      const double x = coords[2 * i];
      const double y = coords[2 * i + 1];
      points.push_back( Point2d{ x, y } );
    }
  }

  template<typename PointLike>
  bool
  point_inside( const PointLike& point ) const
  {
    bool        inside      = false;
    std::size_t point_count = points.size();
    if( point_count < 3 )
    {
      return false;
    }
    for( std::size_t i = 0, j = point_count - 1; i < point_count; j = i++ )
    {
      bool y_in_range = ( ( points[i].y > point.y ) != ( points[j].y > point.y ) );
      if( y_in_range )
      {
        double x_intersect = ( points[j].x - points[i].x ) * ( point.y - points[i].y ) / ( points[j].y - points[i].y ) + points[i].x;

        if( point.x < x_intersect )
        {
          inside = !inside;
        }
      }
    }

    return inside;
  }
};

} // namespace math
} // namespace adore
