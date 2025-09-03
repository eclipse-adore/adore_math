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
