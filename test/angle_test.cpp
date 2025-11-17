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

#include <cmath>
#include <gtest/gtest.h>

#include <adore_math/angles.h>

TEST( Angles, DegreesToRadiansAndBack )
{
  const double deg = 100.0;
  const double rad = adore::math::to_radians( deg );

  EXPECT_NEAR( adore::math::to_degrees( rad ), deg, 1e-9 );
}

TEST( Angles, LatLonToUtmAndBack )
{
  const double latlon_deg = 52.0; // arbitrary
  const double utm_rad    = adore::math::latlon_deg_to_utm_rad( latlon_deg );
  const double back_deg   = adore::math::utm_rad_to_latlon_deg( utm_rad );

  EXPECT_NEAR( back_deg, latlon_deg, 1e-9 );
}
