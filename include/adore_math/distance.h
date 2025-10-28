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
#include <math.h>
#include <cstddef>
#include <limits>
#include <optional>

#include <adore_math/polygon.h>

namespace adore
{
namespace math
{

template<typename PointType, typename OtherPointType>
double
squared_distance_2d( const PointType& a, const OtherPointType& b )
{
  return ( a.x - b.x ) * ( a.x - b.x ) + ( a.y - b.y ) * ( a.y - b.y );
}

template<typename PointType, typename OtherPointType>
double
distance_2d( const PointType& a, const OtherPointType& b )
{
  return std::sqrt( squared_distance_2d( a, b ) );
}

double polygon_distance( const Polygon2d& a, const Polygon2d& b );

double distance_point_to_segment( const Point2d& point, const Point2d& seg_start, const Point2d& seg_end );

template<typename PointType, typename OtherPointType>
std::optional<OtherPointType>
find_closest_point(const PointType& a, const std::vector<OtherPointType>& b)
{
  double closest_observed_distance_sqaured = std::numeric_limits<double>::max();
  std::optional<OtherPointType> closest_point = std::nullopt;

  for( const OtherPointType& p : b)
  {
    double squared_distance_between_points = squared_distance_2d(a, p);
    if ( squared_distance_between_points < closest_observed_distance_sqaured )
    {
      closest_observed_distance_sqaured = squared_distance_between_points;
      closest_point = p;
    }
  }

  return closest_point;
}

} // namespace math
} // namespace adore
