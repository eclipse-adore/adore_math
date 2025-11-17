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

namespace adore

{
namespace math
{


struct Point2d
{
  double x, y;
  Point2d() {};
  Point2d( double x_, double y_ ) :
    x( x_ ),
    y( y_ ) {};
};
} // namespace math
} // namespace adore
