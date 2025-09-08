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

#ifndef MATRIX_CONVERSION_H
#define MATRIX_CONVERSION_H

#include <vector>

#include <Eigen/Dense>

namespace adore
{
namespace math
{
template<typename T>
std::vector<std::vector<T>> to_vector( const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& eigenMatrix );

template<typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> to_eigen( const std::vector<std::vector<T>>& stdVector );
} // namespace math
} // namespace adore

#endif // MATRIX_CONVERSION_H
