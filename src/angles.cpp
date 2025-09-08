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
