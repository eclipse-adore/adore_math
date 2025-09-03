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
} // namespace math
} // namespace adore
