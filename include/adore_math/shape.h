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

#include <iostream>

namespace adore
{
namespace math
{


struct Box3d
{
  // Dimensions
  double length;
  double width;
  double height;

  Box3d() {};
  Box3d( double length, double width, double height ) :
    length( length ),
    width( width ),
    height( height ) {};
};
} // namespace math
} // namespace adore