// This file is part of REVE - Radar Ego Velocity Estimator
// Copyright (C) 2021  Christopher Doer <christopher.doer@kit.edu>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <Eigen/Dense>

// Fortran macros
#if defined(NO_APPEND_FORTRAN)
#  if defined(UPPERCASE_FORTRAN)
#    define F_FUNC(f, F) F
#  else
#    define F_FUNC(f, F) f
#  endif
#else
#  if defined(UPPERCASE_FORTRAN)
#    define F_FUNC(f, F) F##_
#  else
#    define F_FUNC(f, F) f##_
#  endif
#endif

namespace reve
{
/**
 * @brief Solves the given linear problem (y = x * beta) using orthogonal distance regression (odrpack)
 * \note This function is general regarding the dimensions of y(n x 1), x(n x m) and beta (p x 1)
 *       This is just an interface for the fortran implementation of odrpack, see src/ordpack/odrpack_manual.pdf
 * @param[in] y            vector of observations, has to be n x 1
 * @param[in] x            matrix of explanatory variables / model, has to be n x m
 * @param[in] sigma_y      sigma of observations, has to be n x 1
 * @param[in] sigma_x      matrix of sigmas n x m whereas each row defines the digonal elements of the covriance
 *                         matrix of the corresponding model parameters
 * @param[in/out] beta     parameter vector, has to be (p x 1), provides the initial values
 * @param[out] sigma_beta  vector containing the estimated sigmas of beta (see WORK(SDI) p.26 odrpack_manual.pdf)
 * @param[out] covariance_sigma  covariace matrix of the estimated beta (see WORK(VCVI) p.26 odrpack_manual.pdf)
 * @return
 */
bool solveODR(const Eigen::VectorXd& y,
              const Eigen::MatrixXd& x,
              const Eigen::VectorXd& sigma_y,
              const Eigen::MatrixXd& sigma_x,
              Eigen::VectorXd& beta,
              Eigen::VectorXd& sigma_beta,
              Eigen::MatrixXd& covariance_sigma);

}  // namespace reve
