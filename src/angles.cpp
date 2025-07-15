/********************************************************************************
 * Copyright (C) 2017-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 ********************************************************************************/
#include "adore_math/angles.h"

namespace adore
{
namespace math
{

double
latlon_deg_to_utm_rad( double latlon_degrees )
{
  return to_radians( 90 - latlon_degrees );
}

double
utm_rad_to_latlon_deg( double utm_radians )
{
  return to_degrees( M_PI / 2 - utm_radians );
}


} // namespace math
} // namespace adore
