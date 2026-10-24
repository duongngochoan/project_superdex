/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <mochi_core/solvers/newton_solver_params.h>
#include <mochi_core/solvers/newton_solver_status.h>
#include <mochi_core/solvers/snle_problem.h>
#include <mochi_core/utils/basic_utils.h>

#include <limits>

namespace mochi {

/**
 * Utility struct implementing line search algorithms. It has no internal state.
 */
template <typename T>
struct LineSearch {
  using Problem = SnleProblem<T>;
  using Status = NewtonSolverStatus<T>;
  using LsParams = LineSearchParams;

  /**
   * No line search. Just take the step.
   */
  static bool None(Problem& problem, Status& status, LsParams const& params);

  /**
   * Simple backtracking line search with improvement in the merit function.
   */
  static bool Simple(Problem& problem, Status& status, LsParams const& params);

  /**
   * Backtracking line search with Armijo condition for improvement.
   */
  static bool Armijo(Problem& problem, Status& status, LsParams const& params);

  /**
   * Backtracking line search with Wolfe weak conditions for improvement.
   */
  static bool WolfeWeak(Problem& problem, Status& status, LsParams const& params);

  /**
   * Backtracking line search with Wolfe strong conditions for improvement.
   */
  static bool WolfeStrong(Problem& problem, Status& status, LsParams const& params);

  /**
   * Backtracking line search with residual norm condition for improvement.
   */
  static bool ResidualNorm(Problem& problem, Status& status, LsParams const& params);

  /**
   * Backtracking line search that accepts either the Armijo condition or the residual norm
   * condition for improvement.
   */
  static bool ArmijoOrResidualNorm(Problem& problem, Status& status, LsParams const& params);

 private:
  /**
   * Slope test of the trials whose merit change is within an estimate of the merit's rounding
   * error.
   *
   * Near convergence, typically in single precision, the merit change of a good step can be
   * smaller than its rounding error, so the merit test can reject the step on rounding error. With
   * E0 and E1 the merits at x and at the trial x + s dx, ε the machine epsilon of T and
   * k = @ref kRoundingErrorFactor, a trial that fails the merit test with
   * |E1 - E0| <= 2 k ε |E0| is accepted if
   * F(x + s dx)·dx <= (2 c1 - 1) F(x)·dx, with F the residual and c1 the Armijo constant of the
   * merit test. This is the Armijo condition on (s / 2) (F(x)·dx + F(x + s dx)·dx), the
   * trapezoidal estimate of E1 - E0, which is exact on quadratic merits and free of the
   * cancellation in E1 - E0: Hager and Zhang's approximate Armijo condition (2005). It applies
   * only to trials that have a finite merit and fail the merit test, and never accepts one whose
   * slope or slope bound is not finite.
   */
  // TODO: Explore applying the slope test also to trials that pass the merit test within the band.
  class RoundingErrorSlopeTest {
   public:
    /**
     * @param[in] merit0 Merit E0 at the start of the line search.
     * @param[in] slope0 F(x)·dx at the start of the line search.
     * @param[in] c1 Armijo constant c1 of the merit test.
     */
    RoundingErrorSlopeTest(double merit0, double slope0, double c1)
        : _merit0(merit0),
          _band(2.0 * kRoundingErrorFactor * std::numeric_limits<T>::epsilon() * Abs(merit0)),
          _slopeBound((2.0 * c1 - 1.0) * slope0) {}

    /**
     * Whether the trial, with finite merit @p merit and the increment holding @p scale times dx,
     * changes the merit within the band and passes the slope test. Assembles the residual if the
     * slope decides.
     */
    [[nodiscard]] bool Passes(Problem& problem, double merit, T scale) const {
      MOCHI_ASSERT_VERBOSE(IsFinite(merit), "The slope test requires a finite merit.");
      if (!IsFinite(_slopeBound) || !NearEqual(merit, _merit0, _band)) {
        return false;
      }
      problem.UpdateResidual(); // No-op if already assembled
      double const slope = problem.GetResidual().Dot(problem.GetIncrement());
      return IsFinite(slope) && slope <= scale * _slopeBound;
    }

   private:
    // The band allows each of the two merits a rounding error of k ε |E0|. |E0| stands in for the
    // rounding magnitudes of the merit's terms, which cancellation within the terms can make
    // orders of magnitude larger, hence the large k.
    // TODO: Explore per-term rounding magnitudes instead of |E0| for the band. See D125958065.
    static constexpr double kRoundingErrorFactor = 1000.0;

    double const _merit0;
    double const _band; // 2 k ε |E0|
    double const _slopeBound; // (2 c1 - 1) F(x)·dx
  };
};

template <typename T>
bool LineSearch<T>::None(
    Problem& problem,
    Status& /*status*/,
    [[maybe_unused]] LsParams const& params) {
  MOCHI_PROFILE_SCOPE();
  MOCHI_ASSERT_VERBOSE(params.type == LineSearchType::None, "Inconsistent line search type.");

  // Update the current solution without performing line search
  problem.UpdateSolution();

  return true;
}

template <typename T>
bool LineSearch<T>::Simple(Problem& problem, Status& status, LsParams const& params) {
  MOCHI_PROFILE_SCOPE();
  MOCHI_ASSERT_VERBOSE(params.type == LineSearchType::Simple, "Inconsistent line search type.");
  MOCHI_ASSERT(
      params.maxIter > 0,
      "At least 1 line search iteration is required. To run without line search, 'None' line search type must be used.");

  // Save current solution
  ColumnVector<T> const sol0(problem.GetSolution());

  // Simple requires no sufficient decrease, hence c1 = 0
  RoundingErrorSlopeTest const slopeTest(
      status.merit, problem.GetResidual().Dot(problem.GetIncrement()), /*c1*/ 0.0);

  bool improved = false;
  auto scale = T(1);
  int& it = status.numLastLSIterDone;
  for (it = 0; it < params.maxIter; ++it) {
    // Compute new candidate solution
    if (it > 0) {
      problem.SetSolution(sol0, /*invokePost*/ false);
      scale *= params.alpha;
      problem.ScaleIncrement(params.alpha);
    }
    problem.UpdateSolution();

    // Compute objective at x + dx position
    problem.UpdateObjective();
    double const merit = problem.GetObjective();

    // Check enough improvement
    if (IsFinite(merit) &&
        (merit <= (status.merit + Abs(status.merit * params.maxRelIncrease)) ||
         slopeTest.Passes(problem, merit, scale))) {
      status.merit = merit;
      improved = true;
      break;
    }
  }

  return improved;
}

template <typename T>
bool LineSearch<T>::Armijo(Problem& problem, Status& status, LsParams const& params) {
  MOCHI_PROFILE_SCOPE();
  MOCHI_ASSERT_VERBOSE(params.type == LineSearchType::Armijo, "Inconsistent line search type.");
  MOCHI_ASSERT(
      params.maxIter > 0,
      "At least 1 line search iteration is required. To run without line search, 'None' line search type must be used.");

  // Save current solution
  ColumnVector<T> const sol0(problem.GetSolution());

  // Compute Armijo factor
  double const dotDx0Res0 = problem.GetResidual().Dot(problem.GetIncrement());
  MOCHI_ASSERT_VERBOSE(dotDx0Res0 <= 0, "Armijo line search requires a descent direction.");
  double const factor0 = dotDx0Res0 * params.wolfe1;
  RoundingErrorSlopeTest const slopeTest(status.merit, dotDx0Res0, params.wolfe1);

  bool improved = false;
  auto scale = T(1);
  int& it = status.numLastLSIterDone;
  for (it = 0; it < params.maxIter; ++it) {
    // Compute new candidate solution
    if (it > 0) {
      problem.SetSolution(sol0, /*invokePost*/ false);
      scale *= params.alpha;
      problem.ScaleIncrement(params.alpha);
    }
    problem.UpdateSolution();

    // Compute merit at x + dx position
    problem.UpdateObjective();
    double const merit = problem.GetObjective();

    // Check enough improvement
    if (IsFinite(merit) &&
        (merit <= status.merit + scale * factor0 || slopeTest.Passes(problem, merit, scale))) {
      status.merit = merit;
      improved = true;
      break;
    }
  }

  return improved;
}

template <typename T>
bool LineSearch<T>::WolfeWeak(Problem& problem, Status& status, LsParams const& params) {
  MOCHI_PROFILE_SCOPE();
  MOCHI_ASSERT_VERBOSE(params.type == LineSearchType::WolfeWeak, "Inconsistent line search type.");
  MOCHI_ASSERT(
      params.maxIter > 0,
      "At least 1 line search iteration is required. To run without line search, 'None' line search type must be used.");

  // Save current solution
  ColumnVector<T> const sol0(problem.GetSolution());

  // Compute Wolfe factors
  double const dotDx0Res0 = problem.GetResidual().Dot(problem.GetIncrement());
  MOCHI_ASSERT_VERBOSE(dotDx0Res0 <= 0, "Wolfe line search requires a descent direction.");
  double const t1 = params.wolfe1 * dotDx0Res0;
  double const t2 = params.wolfe2 * dotDx0Res0;
  RoundingErrorSlopeTest const slopeTest(status.merit, dotDx0Res0, params.wolfe1);

  bool improved = false;
  auto scale = T(1);
  auto scaleNew = scale;
  auto scaleMin = T(0);
  auto scaleMax = std::numeric_limits<T>::infinity();
  int& it = status.numLastLSIterDone;
  for (it = 0; it < params.maxIter; ++it) {
    MOCHI_ASSERT(
        scaleMin <= scaleMax, "Inconsistent scale bounds: [%.5e, %.5e].\n", scaleMin, scaleMax);

    // Compute new candidate solution
    if (it > 0) {
      problem.SetSolution(sol0, /*invokePost*/ false);
      problem.ScaleIncrement(scaleNew / scale);
      scale = scaleNew;
    }
    problem.UpdateSolution();

    // Compute merit and gradient at x + dx position
    problem.UpdateObjResDRes(
        {.assemObj = true, .assemRes = true, .assemDRes = false, .psdDRes = false});

    // Compute values for Wolfe conditions
    double const dotDxRes = problem.GetResidual().Dot(problem.GetIncrement());
    double const merit = problem.GetObjective();

    // Check current iterate
    if (!IsFinite(dotDxRes) || !IsFinite(merit) ||
        (merit > status.merit + scale * t1 && !slopeTest.Passes(problem, merit, scale))) {
      // If not finite, or neither the 1st Wolfe condition nor the slope test is met, reduce the
      // scale.
      scaleMax = scale;
      scaleNew = scaleMin + params.alpha * (scale - scaleMin);
    } else if (dotDxRes < t2 * scale) {
      // If the 2nd Wolfe condition is not met, increase the scale.
      scaleMin = scale;
      scaleNew = IsFinite(scaleMax) ? T(0.5) * (scale + scaleMax) : scale / params.alpha;
    } else {
      // Otherwise, update merit and exit line search.
      status.merit = merit;
      improved = true;
      break;
    }
  }

  return improved;
}

template <typename T>
bool LineSearch<T>::WolfeStrong(Problem& problem, Status& status, LsParams const& params) {
  MOCHI_PROFILE_SCOPE();
  MOCHI_ASSERT_VERBOSE(
      params.type == LineSearchType::WolfeStrong, "Inconsistent line search type.");
  MOCHI_ASSERT(
      params.maxIter > 0,
      "At least 1 line search iteration is required. To run without line search, 'None' line search type must be used.");

  // Save current solution
  ColumnVector<T> const sol0(problem.GetSolution());

  // Compute Wolfe factors
  double const dotDx0Res0 = problem.GetResidual().Dot(problem.GetIncrement());
  MOCHI_ASSERT_VERBOSE(dotDx0Res0 <= 0, "Wolfe line search requires a descent direction.");
  double const t1 = params.wolfe1 * dotDx0Res0;
  double const t2 = params.wolfe2 * dotDx0Res0;
  RoundingErrorSlopeTest const slopeTest(status.merit, dotDx0Res0, params.wolfe1);

  bool improved = false;
  auto scale = T(1);
  auto scaleNew = scale;
  auto scaleMin = T(0);
  auto scaleMax = std::numeric_limits<T>::infinity();
  int& it = status.numLastLSIterDone;
  for (it = 0; it < params.maxIter; ++it) {
    MOCHI_ASSERT(
        scaleMin <= scaleMax, "Inconsistent scale bounds: [%.5e, %.5e].\n", scaleMin, scaleMax);

    // Compute new candidate solution
    if (it > 0) {
      problem.SetSolution(sol0, /*invokePost*/ false);
      problem.ScaleIncrement(scaleNew / scale);
      scale = scaleNew;
    }
    problem.UpdateSolution();

    // Compute merit and gradient at x + dx position
    problem.UpdateObjResDRes(
        {.assemObj = true, .assemRes = true, .assemDRes = false, .psdDRes = false});

    // Compute values for Wolfe conditions
    double const dotDxRes = problem.GetResidual().Dot(problem.GetIncrement());
    double const merit = problem.GetObjective();

    if (!IsFinite(dotDxRes) || !IsFinite(merit) ||
        (merit > status.merit + scale * t1 && !slopeTest.Passes(problem, merit, scale)) ||
        (dotDxRes > Abs(t2 * scale))) {
      // If not finite, or neither the 1st Wolfe condition nor the slope test is met, or the 2nd
      // Wolfe condition is not met by above, reduce the scale.
      scaleMax = scale;
      scaleNew = scaleMin + params.alpha * (scale - scaleMin);
    } else if (dotDxRes < -Abs(t2 * scale)) {
      // If 2nd Wolfe condition is not met by below, increase the scale.
      scaleMin = scale;
      scaleNew = IsFinite(scaleMax) ? T(0.5) * (scale + scaleMax) : scale / params.alpha;
    } else {
      // Otherwise, update merit and exit line search.
      status.merit = merit;
      improved = true;
      break;
    }
  }

  return improved;
}

template <typename T>
bool LineSearch<T>::ResidualNorm(Problem& problem, Status& status, LsParams const& params) {
  MOCHI_PROFILE_SCOPE();
  MOCHI_ASSERT_VERBOSE(
      params.type == LineSearchType::ResidualNorm, "Inconsistent line search type.");
  MOCHI_ASSERT(
      params.maxIter > 0,
      "At least 1 line search iteration is required. To run without line search, 'None' line search type must be used.");

  // Save current solution
  ColumnVector<T> const sol0(problem.GetSolution());

  // Compute current residual norm
  auto const resNorm0 = problem.GetResidual().Norm();

  bool improved = false;
  int& it = status.numLastLSIterDone;
  for (it = 0; it < params.maxIter; ++it) {
    // Compute new candidate solution
    if (it > 0) {
      problem.SetSolution(sol0, /*invokePost*/ false);
      problem.ScaleIncrement(params.alpha);
    }
    problem.UpdateSolution();

    // Compute the residual at x + dx
    problem.UpdateResidual();
    auto const resNorm = problem.GetResidual().Norm();

    if (IsFinite(resNorm) && resNorm <= resNorm0) {
      improved = true;
      break;
    }
  }

  return improved;
}

template <typename T>
bool LineSearch<T>::ArmijoOrResidualNorm(Problem& problem, Status& status, LsParams const& params) {
  MOCHI_PROFILE_SCOPE();
  MOCHI_ASSERT_VERBOSE(
      params.type == LineSearchType::ArmijoOrResidualNorm, "Inconsistent line search type.");
  MOCHI_ASSERT(
      params.maxIter > 0,
      "At least 1 line search iteration is required. To run without line search, 'None' line search type must be used.");

  // Compute Armijo factor. Armijo requires a descent direction. If it's not a descent direction,
  // fall back to residual norm.
  double const dotDx0Res0 = problem.GetResidual().Dot(problem.GetIncrement());
  if (dotDx0Res0 > 0) {
    auto localParams = params;
    localParams.type = LineSearchType::ResidualNorm;
    return ResidualNorm(problem, status, localParams);
  }
  double const factor0 = dotDx0Res0 * params.wolfe1;
  RoundingErrorSlopeTest const slopeTest(status.merit, dotDx0Res0, params.wolfe1);

  // Save current solution
  ColumnVector<T> const sol0(problem.GetSolution());

  // Compute current residual norm
  auto const resNorm0 = problem.GetResidual().Norm();
  auto const residualNormDecreased = [&] {
    auto const resNorm = problem.GetResidual().Norm();
    return IsFinite(resNorm) && resNorm < resNorm0;
  };

  bool improved = false;
  auto scale = T(1);
  int& it = status.numLastLSIterDone;
  for (it = 0; it < params.maxIter; ++it) {
    // Compute new candidate solution
    if (it > 0) {
      problem.SetSolution(sol0, /*invokePost*/ false);
      scale *= params.alpha;
      problem.ScaleIncrement(params.alpha);
    }
    problem.UpdateSolution();

    // Compute merit and residual at x + dx position, both in the same pass. Even if the residual is
    // not needed here, it is likely needed by the caller.
    problem.UpdateObjResDRes(
        AssemblyParams{.assemObj = true, .assemRes = true, .assemDRes = false});

    // Check improvement of the residual norm or enough improvement of the merit
    double const merit = problem.GetObjective();
    if ((IsFinite(merit) &&
         (merit <= status.merit + scale * factor0 || slopeTest.Passes(problem, merit, scale))) ||
        residualNormDecreased()) {
      status.merit = merit;
      improved = true;
      break;
    }
  }

  return improved;
}

} // namespace mochi
