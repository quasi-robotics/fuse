/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef FUSE_CORE_LOCAL_PARAMETERIZATION_H
#define FUSE_CORE_LOCAL_PARAMETERIZATION_H

#include <fuse_core/ceres_macros.h>
#include <fuse_core/fuse_macros.h>
#include <fuse_core/serialization.h>

#include <boost/serialization/access.hpp>

#if !CERES_SUPPORTS_MANIFOLDS
// Local parameterizations is removed in favour of Manifold in
// version 2.2.0, see
// https://github.com/ceres-solver/ceres-solver/commit/0141ca090c315db2f3c38e1731f0fe9754a4e4cc
#include <ceres/local_parameterization.h>
#endif

namespace fuse_core
{
/**
 * @brief The LocalParameterization interface definition.
 *
 * This class extends the Ceres LocalParameterization class, adding the additional requirement of a
 * \p Minus() method, the conceptual inverse of the already required \p Plus() method.
 *
 * If Plus(x1, delta) -> x2, then Minus(x1, x2) -> delta
 *
 * See the Ceres documentation for more details. http://ceres-solver.org/nnls_modeling.html#localparameterization
 */
class LocalParameterization
// extend ceres LocalParameterization if we are <= 2.1
#if !CERES_SUPPORTS_MANIFOLDS
  : public ceres::LocalParameterization
#endif
{
public:
  FUSE_SMART_PTR_ALIASES_ONLY(LocalParameterization);

  /**
   * @brief Generalization of the subtraction operation
   *
   * Minus(y, x) -> x_minus_y
   *
   * with the conditions that:
   *  - Minus(x, x) -> 0
   *  - if Plus(y, delta) -> x, then Minus(y, x) -> delta
   * 
   * i.e. x + delta = y
   *
   * @param[in]  x    The value of the first variable, of size \p GlobalSize()
   * @param[in]  y    The value of the second variable, of size \p GlobalSize()
   * @param[out] y_minus_x The difference between the second variable and the first, of size \p LocalSize()
   * @return True if successful, false otherwise
   */
  virtual bool Minus(const double* x, const double* y, double* y_minus_x) const = 0;

  /**
   * @brief The jacobian of Minus(y, x) w.r.t x at x = y, i.e
   * 
   *      (D_1 Minus) (x, x)
   *
   * @param[in]  x        The value used to evaluate the Jacobian, of size \p GlobalSize()
   * @param[out] jacobian The first-order derivative in row-major order, of size \p LocalSize() x \p GlobalSize()
   * @return True if successful, false otherwise
   */
  virtual bool ComputeMinusJacobian(const double* x, double* jacobian) const = 0;

#if CERES_SUPPORTS_MANIFOLDS
  virtual ~LocalParameterization() = default;

  /**
   * @brief Generalization of the addition operation,
   * 
   *    x_plus_delta = Plus(x, delta)
   * 
   * with the condition that Plus(x, 0) = x.
   * @param[in] x variable of size \p GlobalSize()
   * @param[in] delta variable of size \p LocalSize()
   * @param[out] x_plus_delta of size \p GlobalSize()
  */
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const = 0;

  /**
   * @brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
   * 
   * @param[in] x variable of size \p GlobalSize()
   * @param[out] jacobian a row-major GlobalSize() x LocalSize() matrix.
   * @return 
   */
  virtual bool ComputeJacobian(const double* x, double* jacobian) const = 0;

  /**
   * @brief Computes local_matrix = global_matrix * jacobian
   *
   * This is only used by GradientProblem. For most normal uses, it is
   * okay to use the default implementation.
   *
   * jacobian(x) is the matrix returned by ComputeJacobian at x.
   *
   * @param[in] x
   * @param[in] num_rows
   * @param[in] global_matrix is a num_rows x GlobalSize  row major matrix.
   * @param[out] local_matrix is a num_rows x LocalSize row major matrix.
   */
  virtual bool MultiplyByJacobian(
    const double* x,
    const int num_rows,
    const double* global_matrix,
    double* local_matrix) const
  {
    if (LocalSize() == 0)
    {
      return true;
    }

    Eigen::MatrixXd jacobian(GlobalSize(), LocalSize());
    if (!ComputeJacobian(x, jacobian.data()))
    {
      return false;
    }

    Eigen::Map<Eigen::MatrixXd>(local_matrix, num_rows, LocalSize()) =
      Eigen::Map<const Eigen::MatrixXd>(global_matrix, num_rows, GlobalSize()) * jacobian;
    return true;
  }

  // Size of x.
  virtual int GlobalSize() const = 0;
  // Size of delta.
  virtual int LocalSize() const = 0;

#endif
private:
  // Allow Boost Serialization access to private methods
  friend class boost::serialization::access;

  /**
   * @brief The Boost Serialize method that serializes all of the data members in to/out of the archive
   *
   * @param[in/out] archive - The archive object that holds the serialized class members
   * @param[in] version - The version of the archive being read/written. Generally unused.
   */
  template <class Archive>
  void serialize(Archive& /* archive */, const unsigned int /* version */)
  {
  }
};

}  // namespace fuse_core

#endif  // FUSE_CORE_LOCAL_PARAMETERIZATION_H
