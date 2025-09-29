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

namespace adore
{
namespace math
{

double constexpr normalize_angle( double angle )
{
  angle = fmod( angle + M_PI, 2.0 * M_PI );
  if( angle < 0 )
    angle += 2.0 * M_PI;
  return angle - M_PI;
}

constexpr double
to_degrees( double radians )
{
  double degrees = radians * ( 180.0 / M_PI );
  degrees        = fmod( degrees, 360.0 );
  if( degrees < 0 )
    degrees += 360.0; // Ensure it's within [0, 360)
  return degrees;
}

constexpr double
to_radians( double degrees )
{
  return normalize_angle( degrees * ( M_PI / 180.0 ) );
}

double latlon_deg_to_utm_rad( double latlon_degrees );
double utm_rad_to_latlon_deg( double utm_radians );

template<typename PointType>
double
compute_yaw( const PointType& from, const PointType& to )
{
  double delta_x = to.x - from.x;
  double delta_y = to.y - from.y;
  return std::atan2( delta_y, delta_x ); // Returns angle in radians
}

// get yaw from quaternion (since tf2 is annoying sometimes)
template<typename Quaternion>
static inline double
get_yaw( const Quaternion& q )
{

  double x = q.x;
  double y = q.y;
  double z = q.z;
  double w = q.w;

  double siny_cosp = 2.0 * ( w * z + x * y );
  double cosy_cosp = 1.0 - 2.0 * ( y * y + z * z );
  double yaw       = std::atan2( siny_cosp, cosy_cosp );

  return yaw;
}

template<typename Quaternion>
static inline double
get_roll( const Quaternion& q )
{
  double x = q.x;
  double y = q.y;
  double z = q.z;
  double w = q.w;

  double sinr_cosp = 2.0 * ( w * x + y * z );
  double cosr_cosp = 1.0 - 2.0 * ( x * x + y * y );
  double roll      = std::atan2( sinr_cosp, cosr_cosp );

  return roll;
}

template<typename Quaternion>
static inline double
get_pitch( const Quaternion& q )
{
  double x = q.x;
  double y = q.y;
  double z = q.z;
  double w = q.w;

  double sinp = 2.0 * ( w * y - z * x );
  double pitch;

  if( std::abs( sinp ) >= 1 )
    pitch = std::copysign( M_PI / 2, sinp ); // Use 90 degrees if out of range
  else
    pitch = std::asin( sinp );

  return pitch;
}
} // namespace math
} // namespace adore
