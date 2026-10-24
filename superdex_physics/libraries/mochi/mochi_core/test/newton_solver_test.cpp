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

#include <mochi_core/solvers/line_search.h>
#include <mochi_core/solvers/newton_solver.h>
#include <mochi_core/test/mochi_test_helpers.h>
#include <mochi_core/utils/sparsity_utils.h>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

using namespace mochi;

// ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯

/**
 * Implements a very simple non-linear problem to test.
 */
static SnleProblem<real> GetTestProblem() {
  // Some SNLE data
  auto residual = ColumnVector<real>{1};
  auto dresidual = AnyMatrix<real>{Matrix<real>{1, 1}};
  SetZero(residual);
  SetZero(dresidual);

  // Capture the SNLE data in an assembly function
  auto assembleFn = [=](SnleProblem<real>& problem, AssemblyParams const& params) mutable {
    auto solution = problem.GetSolution();
    real x = solution[0];

    // Evaluate f(x) and ∂f/∂x(x)
    real const fx = -(x * x) / 10.0_r - x * Cos(x); // gradient
    real const dfdx = -x / 5.0_r - Cos(x) + x * Sin(x); // Hessian
    double objective = 0.0;
    if (params.assemObj) {
      objective = -(x * x * x) / 30.0_r - x * Sin(x) - Cos(x);
    }
    if (params.assemRes) {
      residual[0] = fx;
    }
    if (params.assemDRes) {
      auto& dresMat = std::get<Matrix<real>>(dresidual);
      dresMat(0, 0) = dfdx;
    }

    // Add SNLE data to the problem, as if it came from a single actor
    if (params.assemObj) {
      problem.objective = objective;
    }
    if (params.assemRes) {
      problem.actorResiduals.resize(1);
      problem.actorResiduals[0] = std::make_pair(0, &residual);
    }
    if (params.assemDRes) {
      problem.actorMatrices.resize(1);
      problem.actorMatrices[0] = std::make_pair(0, &dresidual);
    }
  };

  // Trivial increment function
  auto incrementFn = [](SnleProblem<real>& problem) { problem.solution += problem.increment; };

  // Create an SnleProblem
  SnleProblemFunctions<real> functions;
  functions.assemble = std::move(assembleFn);
  functions.onPostNewIncrement = std::move(incrementFn);
  int constexpr kNumDofs = 1;
  return SnleProblem<real>{kNumDofs, kNumDofs, std::move(functions)};
}

/**
 * Broyden-like problem
 */
static SnleProblem<real> GetBroydenProblem(int n) {
  // Sparsity
  std::vector<NdArray<int, 2>> spPattern;
  for (int i = 0; i < n; ++i) {
    if (i > 0) {
      spPattern.emplace_back(i, i - 1);
    }
    spPattern.emplace_back(i, i);
    if (i + 1 < n) {
      spPattern.emplace_back(i, i + 1);
    }
  }

  // Some SNLE data
  auto residual = ColumnVector<real>{n};
  auto dresidual = AnyMatrix<real>{SparseMatrix<real>{MakeSparsityGraph(spPattern)}};
  SetZero(residual);
  SetZero(dresidual);

  // Capture the SNLE data in an assembly function
  auto assembleFn = [=](SnleProblem<real>& problem, AssemblyParams const& params) mutable {
    auto solution = problem.GetSolution();
    if (params.assemObj) {
      double objective = 0.0;
      for (int i = 0; i < n; ++i) {
        objective += solution[i] *
            (1_r + (3_r / 2_r) * solution[i] - (1_r / 6_r) * solution[i] * solution[i]);
      }
      for (int i = 0; i < n - 1; ++i) {
        objective -= 2_r * solution[i] * solution[i + 1];
      }
      problem.objective = objective;
    }

    if (params.assemRes) {
      MOCHI_ASSERT(residual.size() == n);
      residual[0] = 1_r + (3_r - 0.5_r * solution[0]) * solution[0] - 2_r * solution[1];
      for (int i = 1; i < n - 1; ++i) {
        residual[i] = 1_r + (3_r - 0.5_r * solution[i]) * solution[i] - 2_r * solution[i - 1] -
            2_r * solution[i + 1];
      }
      residual[n - 1] =
          1_r + (3_r - 0.5_r * solution[n - 1]) * solution[n - 1] - 2_r * solution[n - 2];
      problem.actorResiduals.resize(1);
      problem.actorResiduals[0] = std::make_pair(0, &residual);
    }

    if (params.assemDRes) {
      auto& dresMat = std::get<SparseMatrix<real>>(dresidual);
      MOCHI_ASSERT(dresMat.Rows() == n);
      for (int i = 0; i < n; ++i) {
        if (i > 0) {
          dresMat.SetValue(i, i - 1, -2_r);
        }
        dresMat.SetValue(i, i, 3_r - solution[i]);
        if (i + 1 < n) {
          dresMat.SetValue(i, i + 1, -2_r);
        }
      }
      problem.actorMatrices.resize(1);
      problem.actorMatrices[0] = std::make_pair(0, &dresidual);
    }
  };

  // Trivial increment function
  auto incrementFn = [](SnleProblem<real>& problem) { problem.solution += problem.increment; };

  // Create an SnleProblem
  SnleProblemFunctions<real> functions;
  functions.assemble = std::move(assembleFn);
  functions.onPostNewIncrement = std::move(incrementFn);
  return SnleProblem<real>{n, n, std::move(functions)};
}

/**
 * Broyden-like problem
 */
static SnleProblem<real> GetBroydenProblem2(int n) {
  // Sparsity
  std::vector<NdArray<int, 2>> spPattern;
  for (int i = 0; i < n; ++i) {
    if (i > 0) {
      spPattern.emplace_back(i, i - 1);
    }
    spPattern.emplace_back(i, i);
    if (i + 1 < n) {
      spPattern.emplace_back(i, i + 1);
    }
  }

  // Some SNLE data
  auto residual = ColumnVector<real>{n};
  auto dresidual = AnyMatrix<real>{SparseMatrix<real>{MakeSparsityGraph(spPattern)}};
  SetZero(residual);
  SetZero(dresidual);

  // Capture the SNLE data in an assembly function
  auto assembleFn = [=](SnleProblem<real>& problem, AssemblyParams const& params) mutable {
    auto solution = problem.GetSolution();
    if (params.assemObj) {
      double objective = 0.0;
      objective += solution[0] + (3_r / 4_r) * solution[0] * solution[0] -
          (1_r / 6_r) * solution[0] * solution[0] * solution[0];
      for (int i = 1; i + 1 < n; ++i) {
        objective += solution[i] + (3_r / 2_r) * solution[i] * solution[i] -
            (1_r / 6_r) * solution[i] * solution[i] * solution[i];
      }
      objective += solution[n - 1] + (3_r / 4_r) * solution[n - 1] * solution[n - 1] -
          (1_r / 6_r) * solution[n - 1] * solution[n - 1] * solution[n - 1];
      for (int i = 0; i < n - 1; ++i) {
        objective -= 1.5_r * solution[i] * solution[i + 1];
      }
      problem.objective = objective;
    }

    if (params.assemRes) {
      MOCHI_ASSERT(residual.size() == n);
      residual[0] =
          1_r + 1.5_r * solution[0] - 0.5_r * solution[0] * solution[0] - 1.5_r * solution[1];
      for (int i = 1; i < n - 1; ++i) {
        residual[i] = 1_r + 3_r * solution[i] - 0.5_r * solution[i] * solution[i] -
            1.5_r * solution[i - 1] - 1.5_r * solution[i + 1];
      }
      residual[n - 1] = 1_r + 1.5_r * solution[n - 1] - 0.5_r * solution[n - 1] * solution[n - 1] -
          1.5_r * solution[n - 2];
      problem.actorResiduals.resize(1);
      problem.actorResiduals[0] = std::make_pair(0, &residual);
    }

    if (params.assemDRes) {
      auto& dresMat = std::get<SparseMatrix<real>>(dresidual);
      MOCHI_ASSERT(dresMat.Rows() == n);
      for (int i = 0; i < n; ++i) {
        if (i > 0) {
          dresMat.SetValue(i, i - 1, -1.5_r);
        }
        if ((i == 0) || (i == n - 1)) {
          dresMat.SetValue(i, i, 1.5_r - solution[i]);
        } else {
          dresMat.SetValue(i, i, 3_r - solution[i]);
        }
        if (i + 1 < n) {
          dresMat.SetValue(i, i + 1, -1.5_r);
        }
      }
      problem.actorMatrices.resize(1);
      problem.actorMatrices[0] = std::make_pair(0, &dresidual);
    }
  };

  // Trivial increment function
  auto incrementFn = [](SnleProblem<real>& problem) { problem.solution += problem.increment; };

  // Create an SnleProblem
  SnleProblemFunctions<real> functions;
  functions.assemble = std::move(assembleFn);
  functions.onPostNewIncrement = std::move(incrementFn);
  return SnleProblem<real>{n, n, std::move(functions)};
}

TEST(NewtonSolver, SmoothLocalMinimum) {
  constexpr auto kEpsilon = std::numeric_limits<real>::epsilon();
  constexpr real kAbsTol = 1e3_r * kEpsilon;

  SnleProblem<real> problem = GetTestProblem();
  for (auto solverType :
       {NonLinearSolverType::Newton, NonLinearSolverType::BFGS, NonLinearSolverType::SR1}) {
    for (auto dResidualAssemblyPeriod : {1, 2, 4, 8}) {
      bool const wasWarningEnabled = IsLogChannelEnabled(LogChannel::Warning);
      if (solverType != NonLinearSolverType::Newton && dResidualAssemblyPeriod == 1) {
        // Disable warnings about quasi-Newton methods with dResidualAssemblyPeriod == 1.
        EnableLogChannel(LogChannel::Warning, false);
      }
      if (solverType != NonLinearSolverType::Newton) {
        // Disable warnings about using the dense LDLT with a low-rank augmented matrix.
        EnableLogChannel(LogChannel::Warning, false);
      }
      MOCHI_DEFER(EnableLogChannel(LogChannel::Warning, wasWarningEnabled));

      for (auto lineSearchType :
           {LineSearchType::None,
            LineSearchType::Simple,
            LineSearchType::Armijo,
            LineSearchType::WolfeWeak,
            LineSearchType::WolfeStrong,
            LineSearchType::ResidualNorm,
            LineSearchType::ArmijoOrResidualNorm}) {
        ColumnVector<real, 1> x0(-3_r); // Hessian is positive definite starting from -3.
        problem.SetSolution(x0);

        // Set linear parameters
        NewtonSolverParams params;
        params.lParams.solverType = LinearSolverType::LDLT;
        params.lParams.preconditionerType = PreconditionerType::None;
        params.lParams.absTol = kAbsTol;
        params.lParams.relTol = kEpsilon;

        // Set nonlinear parameters
        params.solverType = solverType;
        params.dResidualAssemblyPeriod = dResidualAssemblyPeriod;
        params.maxIter = 200;
        params.lineSearch.type = lineSearchType;
        params.psdProjMode = PsdProjectionMode::Never;
        params.convergenceMode = NonLinearSolverConvergenceMode::Global;
        params.absTolRes = kAbsTol;
        params.relTolRes = kEpsilon;
        params.solRelTol = 0_r;

        // Create solver
        NewtonSolver<real> solver(params);

        // Solver problem
        auto result = solver.Solve(problem);

        // Expected solution
        double const ref = -1.4275517787645941208;
        double const obj = -ref * ref * ref / 30.0 - ref * Sin(ref) - Cos(ref);

        EXPECT_TRUE(result.convergence == ConvergenceStatus::Converged);
        if (params.lineSearch.type == LineSearchType::None ||
            params.lineSearch.type == LineSearchType::ResidualNorm) {
          // None and ResidualNorm do not update the objective
          EXPECT_NEAR(result.merit, 0_r, kAbsTol);
        } else {
          EXPECT_NEAR(result.merit, obj, kAbsTol);
        }

        EXPECT_NEAR(problem.GetSolution()[0], real(ref), kAbsTol);

        problem.UpdateObjective();
        EXPECT_NEAR(problem.GetObjective(), real(obj), kAbsTol);

        problem.SetSolution(x0);
        EXPECT_TRUE(test::NearEqualMatrices(problem.GetSolution(), x0));

        {
          problem.UpdateObjective();
          double obj0 = -x0[0] * x0[0] * x0[0] / 30.0 - x0[0] * Sin(x0[0]) - Cos(x0[0]);
          EXPECT_NEAR(problem.GetObjective(), real(obj0), kAbsTol);
        }

        problem.UpdateResidual();
        {
          double res0 = -x0[0] * x0[0] / 10.0 - x0[0] * Cos(x0[0]);
          EXPECT_NEAR(problem.GetResidual()[0], real(res0), kAbsTol);
        }

        auto resultBis = solver.Solve(problem);
        EXPECT_NEAR(problem.GetSolution()[0], real(ref), kAbsTol);

        problem.UpdateObjective();
        EXPECT_NEAR(problem.GetObjective(), real(obj), kAbsTol);
        EXPECT_EQ(result.convergence, resultBis.convergence);
        EXPECT_EQ(result.numIterDone, resultBis.numIterDone);
        EXPECT_EQ(result.numLastLSIterDone, resultBis.numLastLSIterDone);
        EXPECT_EQ(result.totalNumLinearIterDone, resultBis.totalNumLinearIterDone);

        // The stored increment must be the step taken, also when the line search fails and the
        // solve keeps its last trial. With these parameters, Armijo and Wolfe reject both trials.
        params.maxIter = 1;
        params.lineSearch.maxIter = 2;
        params.lineSearch.alpha = 0.5_r;
        params.lineSearch.wolfe1 = 0.75_r;
        solver.SetParams(params);
        problem.SetSolution(x0);
        solver.Solve(problem);
        EXPECT_NEAR(problem.GetIncrement()[0], problem.GetSolution()[0] - x0[0], kAbsTol);
      }
    }
  }
}

TEST(NewtonSolver, QuasiNewton) {
  // Verify quasi-Newton updates help convergence on multi-dimensional Broyden problems.
  constexpr real kEpsilon = std::numeric_limits<real>::epsilon();
  constexpr real kAbsTol = 1e3_r * kEpsilon;

  constexpr int n = 50;
  constexpr int period = 8;

  SnleProblem<real> problem = GetBroydenProblem(n);
  ColumnVector<real> x0(n);
  x0.SetConstant(-3.0_r);

  NewtonSolverParams params;
  params.lParams.solverType = LinearSolverType::CG;
  params.lParams.preconditionerType = PreconditionerType::None;
  params.lParams.normType = LinearSolverConvergenceNorm::ResidualL2;
  params.lParams.absTol = 0.1_r * kAbsTol;
  params.lParams.relTol = 0.1_r * kEpsilon;
  params.linearToleranceStrategy = LinearToleranceStrategy::Constant;
  params.dResidualAssemblyPeriod = period;
  params.maxIter = 200;
  params.lineSearch.type = LineSearchType::ResidualNorm;
  params.psdProjMode = PsdProjectionMode::Never;
  params.convergenceMode = NonLinearSolverConvergenceMode::Global;
  params.absTolRes = kAbsTol;
  params.relTolRes = kEpsilon;
  params.solRelTol = 0_r;

  // Baseline: frozen Newton (no Hessian updates between reassemblies)
  problem.SetSolution(x0);
  params.solverType = NonLinearSolverType::Newton;
  NewtonSolver<real> solver(params);
  auto resultNewton = solver.Solve(problem);
  EXPECT_EQ(resultNewton.convergence, ConvergenceStatus::Converged);

  // BFGS should converge in fewer iterations than frozen Newton
  for (auto solverType : {NonLinearSolverType::BFGS, NonLinearSolverType::SR1}) {
    problem.SetSolution(x0);
    params.solverType = solverType;
    solver.SetParams(params);
    auto resultQuasiNewton = solver.Solve(problem);
    EXPECT_EQ(ConvergenceStatus::Converged, resultQuasiNewton.convergence);
    EXPECT_LT(resultQuasiNewton.numIterDone, resultNewton.numIterDone);
  }
}

TEST(NewtonSolver, LinearToleranceStrategy) {
  constexpr real kEpsilon = std::numeric_limits<real>::epsilon();
  constexpr real kRelTol = kEpsilon;
  constexpr real kAbsTol = 1e3_r * kEpsilon;

  // Set up the solver parameters.
  NewtonSolverParams params;
  params.lParams.solverType = LinearSolverType::CG;
  params.lParams.preconditionerType = PreconditionerType::None;
  params.lParams.normType = LinearSolverConvergenceNorm::ResidualL2;
  params.lParams.absTol = 0.1_r * kAbsTol;
  params.lParams.relTol = 0.1_r * kRelTol;
  params.maxIter = 200;
  params.lineSearch.type = LineSearchType::ResidualNorm;
  params.psdProjMode = PsdProjectionMode::Never;
  params.convergenceMode = NonLinearSolverConvergenceMode::Global;
  params.absTolRes = kAbsTol;
  params.relTolRes = kRelTol;
  params.solRelTol = 0_r;

  // Set up the solver.
  NewtonSolver<real> solver(params);

  // Loop over Broyden problems of multiple sizes. Consider only large sizes so that the problem is
  // sufficiently hard for Eisenstat-Walker to converge in strictly fewer iterations than constant.
  for (int n : {100, 500, 1000}) {
    SnleProblem<real> problem = GetBroydenProblem(n);
    ColumnVector<real> x0(n);
    x0.SetConstant(-3.0_r);

    // Solve the problem with constant linear tolerance strategy.
    problem.SetSolution(x0);

    params.linearToleranceStrategy = LinearToleranceStrategy::Constant;
    solver.SetParams(params);
    auto const resultConstant = solver.Solve(problem);

    // Solve the problem with Eisenstat-Walker choice #1 linear tolerance strategy.
    problem.SetSolution(x0);

    params.linearToleranceStrategy = LinearToleranceStrategy::EisenstatWalker1;
    solver.SetParams(params);
    auto const resultEW1 = solver.Solve(problem);

    // Solve the problem with Eisenstat-Walker choice #2 linear tolerance strategy.
    problem.SetSolution(x0);

    params.linearToleranceStrategy = LinearToleranceStrategy::EisenstatWalker2;
    solver.SetParams(params);
    auto const resultEW2 = solver.Solve(problem);

    // Check the solver converged
    EXPECT_EQ(resultConstant.convergence, ConvergenceStatus::Converged);
    EXPECT_EQ(resultEW1.convergence, ConvergenceStatus::Converged);
    EXPECT_EQ(resultEW2.convergence, ConvergenceStatus::Converged);

    if (params.lineSearch.type == LineSearchType::ResidualNorm) {
      // The objective function is not updated
      EXPECT_NEAR(resultConstant.merit, 0_r, kAbsTol);
      EXPECT_NEAR(resultEW1.merit, 0_r, kAbsTol);
      EXPECT_NEAR(resultEW2.merit, 0_r, kAbsTol);
    }

    // Check the total number of linear iterations is smaller with Eisenstat-Walker strategies.
    EXPECT_LT(resultEW1.totalNumLinearIterDone, resultConstant.totalNumLinearIterDone);
    EXPECT_LT(resultEW2.totalNumLinearIterDone, resultConstant.totalNumLinearIterDone);
  }
}

TEST(NewtonSolver, LinearToleranceStrategy2) {
  constexpr real kEpsilon = std::numeric_limits<real>::epsilon();
  constexpr real kRelTol = kEpsilon;
  constexpr real kAbsTol = 1e3_r * kEpsilon;

  // Set up the solver parameters.
  NewtonSolverParams params;
  params.lParams.solverType = LinearSolverType::CG;
  params.lParams.preconditionerType = PreconditionerType::None;
  params.lParams.normType = LinearSolverConvergenceNorm::ResidualL2;
  params.lParams.absTol = 0.1_r * kAbsTol;
  params.lParams.relTol = 0.1_r * kRelTol;
  params.maxIter = 200;
  params.lineSearch.type = LineSearchType::ResidualNorm;
  params.psdProjMode = PsdProjectionMode::Never;
  params.convergenceMode = NonLinearSolverConvergenceMode::Global;
  params.absTolRes = kAbsTol;
  params.relTolRes = kRelTol;
  params.solRelTol = 0_r;

  // Set up the solver.
  NewtonSolver<real> solver(params);

  int n = 1234;

  SnleProblem<real> problem = GetBroydenProblem2(n);
  ColumnVector<real> x0(n);
  x0.SetConstant(-3.0_r);

  // Solve the problem with constant linear tolerance strategy.
  problem.SetSolution(x0);

  params.linearToleranceStrategy = LinearToleranceStrategy::Constant;
  solver.SetParams(params);
  auto const resultConstant = solver.Solve(problem);

  {
    problem.SetSolution(x0);
    problem.UpdateObjective();
    problem.UpdateResidual();
    auto resultConstantBis = solver.Solve(problem);
    EXPECT_EQ(resultConstant.convergence, resultConstantBis.convergence);
    EXPECT_EQ(resultConstant.numIterDone, resultConstantBis.numIterDone);
    EXPECT_EQ(resultConstant.numLastLSIterDone, resultConstantBis.numLastLSIterDone);
    EXPECT_EQ(resultConstant.totalNumLinearIterDone, resultConstantBis.totalNumLinearIterDone);
  }

  // Solve the problem with Eisenstat-Walker choice #1 linear tolerance strategy.
  problem.SetSolution(x0);
  params.linearToleranceStrategy = LinearToleranceStrategy::EisenstatWalker1;
  solver.SetParams(params);
  auto const resultEW1 = solver.Solve(problem);
  {
    problem.SetSolution(x0);
    problem.UpdateObjective();
    problem.UpdateResidual();
    auto resultEW1Bis = solver.Solve(problem);
    EXPECT_EQ(resultEW1.convergence, resultEW1Bis.convergence);
    EXPECT_EQ(resultEW1.numIterDone, resultEW1Bis.numIterDone);
    EXPECT_EQ(resultEW1.numLastLSIterDone, resultEW1Bis.numLastLSIterDone);
    EXPECT_EQ(resultEW1.totalNumLinearIterDone, resultEW1Bis.totalNumLinearIterDone);
  }

  // Solve the problem with Eisenstat-Walker choice #2 linear tolerance strategy.
  problem.SetSolution(x0);
  params.linearToleranceStrategy = LinearToleranceStrategy::EisenstatWalker2;
  solver.SetParams(params);
  auto const resultEW2 = solver.Solve(problem);
  {
    problem.SetSolution(x0);
    problem.UpdateObjective();
    problem.UpdateResidual();
    auto resultEW2Bis = solver.Solve(problem);
    EXPECT_EQ(resultEW2.convergence, resultEW2Bis.convergence);
    EXPECT_EQ(resultEW2.numIterDone, resultEW2Bis.numIterDone);
    EXPECT_EQ(resultEW2.numLastLSIterDone, resultEW2Bis.numLastLSIterDone);
    EXPECT_EQ(resultEW2.totalNumLinearIterDone, resultEW2Bis.totalNumLinearIterDone);
  }

  // Check the solver converged
  EXPECT_EQ(resultConstant.convergence, ConvergenceStatus::Converged);
  EXPECT_EQ(resultEW1.convergence, ConvergenceStatus::Converged);
  EXPECT_EQ(resultEW2.convergence, ConvergenceStatus::Converged);

  if (params.lineSearch.type == LineSearchType::ResidualNorm) {
    // The objective function is not updated
    EXPECT_NEAR(resultConstant.merit, 0_r, kAbsTol);
    EXPECT_NEAR(resultEW1.merit, 0_r, kAbsTol);
    EXPECT_NEAR(resultEW2.merit, 0_r, kAbsTol);
  }

  // This structured problem can converge in one CG iteration per Newton step for every strategy.
  // Eisenstat-Walker must not require more linear iterations, but may tie the constant strategy.
  EXPECT_LE(resultEW1.totalNumLinearIterDone, resultConstant.totalNumLinearIterDone);
  EXPECT_LE(resultEW2.totalNumLinearIterDone, resultConstant.totalNumLinearIterDone);

  ColumnVector<real> xref(n);
  xref.SetConstant(-Sqrt(2_r));
  EXPECT_TRUE(test::NearEqualMatrices(problem.GetSolution(), xref));
}

// ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯

/**
 * Two-actor problem with mixed linear/nonlinear actors for testing PerActorWeighted convergence.
 * Actor 0 owns DoF 0: r₀ = a·x₀ - b         (linear, converges in 1 Newton iteration)
 * Actor 1 owns DoF 1: r₁ = x₁² - c          (nonlinear, solution x₁ = √c)
 * The two actors are decoupled (diagonal dresidual).
 *
 * If populateConvergenceWeights is false, actorConvergenceWeights is left empty so that
 * PerActorWeighted mode triggers the size-mismatch fallback-to-Global path.
 */
static SnleProblem<real> GetTwoActorMixedProblem(
    ColumnVector<real> const& weight0,
    ColumnVector<real> const& weight1,
    bool populateConvergenceWeights = true,
    bool reverseActorOrder = false) {
  auto residual0 = ColumnVector<real>::Zero(1);
  auto residual1 = ColumnVector<real>::Zero(1);
  auto dresidual = AnyMatrix<real>{Matrix<real>::Zero(2, 2)};

  auto assembleFn = [=, &weight0, &weight1](
                        SnleProblem<real>& problem, AssemblyParams const& params) mutable {
    auto const x = problem.GetSolution();
    if (params.assemObj) {
      problem.objective = 0.5 * x[0] * x[0] - 2_r * x[0] + x[1] * x[1] * x[1] / 3.0 - 2_r * x[1];
    }
    if (params.assemRes) {
      residual0[0] = x[0] - 2_r;
      residual1[0] = x[1] * x[1] - 2_r;
      problem.actorResiduals.clear();
      problem.actorConvergenceWeights.clear();
      auto addActor =
          [&](int offset, ColumnVector<real>& residual, ColumnVector<real> const& weight) {
            problem.actorResiduals.emplace_back(offset, &residual);
            if (populateConvergenceWeights) {
              problem.actorConvergenceWeights.emplace_back(offset, &weight);
            }
          };
      if (reverseActorOrder) {
        addActor(1, residual1, weight1);
        addActor(0, residual0, weight0);
      } else {
        addActor(0, residual0, weight0);
        addActor(1, residual1, weight1);
      }
    }
    if (params.assemDRes) {
      auto& d = std::get<Matrix<real>>(dresidual);
      d(0, 0) = 1_r;
      d(1, 1) = 2_r * x[1];
      d(0, 1) = 0_r;
      d(1, 0) = 0_r;
      problem.actorMatrices.clear();
      problem.actorMatrices.emplace_back(0, &dresidual);
    }
  };

  SnleProblemFunctions<real> functions;
  functions.assemble = std::move(assembleFn);
  functions.onPostNewIncrement = [](SnleProblem<real>& p) { p.solution += p.increment; };
  return SnleProblem<real>{2, 2, std::move(functions)};
}

/**
 * Actor 0 converges via absTol in 1 iteration, actor 1 converges via relTol in 1 iteration.
 * Actor 0: linear r₀ = x₀ - 2, from x₀=0: after 1 step r₀ = 0 ≤ absTol.
 * Actor 1: nonlinear r₁ = x₁² - 2, from x₁=2: after 1 step x₁=1.5, r₁=0.25.
 *   Initial weighted norm = 2, post-step weighted norm = 0.25. Ratio = 0.125 ≤ relTol = 0.2.
 *   But 0.25 > absTol = 0.1, so actor 1 does NOT converge via absTol.
 */
TEST(NewtonSolver, PerActorWeighted_AbsTolAndRelTol) {
  auto w0 = ColumnVector<real>::Ones(1);
  auto w1 = ColumnVector<real>::Ones(1);

  auto problem = GetTwoActorMixedProblem(w0, w1);
  problem.SetSolution(ColumnVector<real>({{0_r, 2_r}}));

  NewtonSolverParams params;
  params.convergenceMode = NonLinearSolverConvergenceMode::PerActorWeighted;
  params.absTolRes = 0.1_r;
  params.relTolRes = 0.2_r;
  params.maxIter = 10;

  NewtonSolver<real> solver(params);
  auto const status = solver.Solve(problem);

  EXPECT_EQ(status.convergence, ConvergenceStatus::Converged);
  ASSERT_EQ(isize(status.actorConvergence), 2);
  EXPECT_EQ(status.actorConvergence[0], ConvergenceStatus::Converged);
  EXPECT_EQ(status.actorConvergence[1], ConvergenceStatus::Converged);
  EXPECT_EQ(status.numIterDone, 1);
}

TEST(NewtonSolver, PerActorWeighted_SortsWeightsWithResiduals) {
  for (bool const reverseActorOrder : {false, true}) {
    auto w0 = ColumnVector<real>::Ones(1);
    auto w1 = ColumnVector<real>::Ones(1);
    w1[0] = 100_r;

    auto problem =
        GetTwoActorMixedProblem(w0, w1, /*populateConvergenceWeights*/ true, reverseActorOrder);
    problem.SetSolution(ColumnVector<real>({{0_r, 2_r}}));

    NewtonSolverParams params;
    params.convergenceMode = NonLinearSolverConvergenceMode::PerActorWeighted;
    params.absTolRes = 3_r;
    params.relTolRes = 0_r;
    params.maxIter = 0;

    NewtonSolver<real> solver(params);
    auto const status = solver.Solve(problem);

    EXPECT_EQ(status.convergence, ConvergenceStatus::Stopped);
    ASSERT_EQ(isize(status.actorConvergence), 2);
    EXPECT_EQ(status.actorConvergence[0], ConvergenceStatus::Converged);
    EXPECT_EQ(status.actorConvergence[1], ConvergenceStatus::Stopped);
  }
}

/**
 * One actor converges, the other does not.
 * Actor 0: linear r₀ = x₀ - 2, from x₀=0: after 1 step r₀ = 0 (converges).
 * Actor 1: nonlinear r₁ = x₁² - 2, from x₁=2: after 1 step r₁ = 0.25.
 *   absTol = 0.1: 0.25 > 0.1 → fails.
 *   relTol = 0.05: 0.25/2.0 = 0.125 > 0.05 → fails.
 */
TEST(NewtonSolver, PerActorWeighted_OneActorFails) {
  auto w0 = ColumnVector<real>::Ones(1);
  auto w1 = ColumnVector<real>::Ones(1);

  auto problem = GetTwoActorMixedProblem(w0, w1);
  problem.SetSolution(ColumnVector<real>({{0_r, 2_r}}));

  NewtonSolverParams params;
  params.convergenceMode = NonLinearSolverConvergenceMode::PerActorWeighted;
  params.absTolRes = 0.1_r;
  params.relTolRes = 0.05_r;
  params.maxIter = 1;

  NewtonSolver<real> solver(params);
  auto const status = solver.Solve(problem);

  EXPECT_EQ(status.convergence, ConvergenceStatus::Stopped);
  ASSERT_EQ(isize(status.actorConvergence), 2);
  EXPECT_EQ(status.actorConvergence[0], ConvergenceStatus::Converged);
  EXPECT_EQ(status.actorConvergence[1], ConvergenceStatus::Stopped);
}

/**
 * PerActorWeighted mode with unpopulated actorConvergenceWeights should fall back to Global
 * convergence checking and converge.
 */
TEST(NewtonSolver, PerActorWeighted_FallbackToGlobal) {
  auto w0 = ColumnVector<real>::Ones(1);
  auto w1 = ColumnVector<real>::Ones(1);

  auto problem = GetTwoActorMixedProblem(w0, w1, /*populateConvergenceWeights*/ false);
  problem.SetSolution(ColumnVector<real>({{0_r, 2_r}}));

  NewtonSolverParams params;
  params.convergenceMode = NonLinearSolverConvergenceMode::PerActorWeighted;
  params.absTolRes = 1e-4_r;
  params.maxIter = 10;

  // Suppress the expected "falling back to Global" warning emitted by the solver.
  bool const wasWarningEnabled = IsLogChannelEnabled(LogChannel::Warning);
  EnableLogChannel(LogChannel::Warning, false);
  MOCHI_DEFER(EnableLogChannel(LogChannel::Warning, wasWarningEnabled));

  NewtonSolver<real> solver(params);
  auto const status = solver.Solve(problem);

  EXPECT_EQ(ConvergenceStatus::Converged, status.convergence); // Solver converged.
  EXPECT_TRUE(status.actorConvergence.empty()); // Empty after fallback.
}

// ⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯⎯

/**
 * The full step lands where the residual is NaN and the objective is NaN or −∞, and must be
 * rejected. The half step is acceptable to every line search.
 */
TEST(NewtonSolver, LineSearchRejectsNonFiniteTrial) {
  real invalidObjective = 0_r;
  auto residual = ColumnVector<real>::Zero(2);
  SnleProblemFunctions<real> functions;
  functions.assemble = [&invalidObjective, &residual](
                           SnleProblem<real>& problem, AssemblyParams const& params) {
    auto const x = problem.GetSolution();
    bool const isInvalid = x[0] < -1_r;
    if (params.assemObj) {
      problem.objective = isInvalid ? invalidObjective : 0.5 * x.Dot(x);
    }
    if (params.assemRes) {
      residual = x;
      if (isInvalid) {
        residual.SetConstant(std::numeric_limits<real>::quiet_NaN());
      }
      problem.actorResiduals = {{0, &residual}};
    }
  };
  functions.onPostNewIncrement = [](SnleProblem<real>& p) { p.solution += p.increment; };
  SnleProblem<real> problem{2, 2, std::move(functions)};

  using Ls = LineSearch<real>;
  for (real const objective :
       {std::numeric_limits<real>::quiet_NaN(), -std::numeric_limits<real>::infinity()}) {
    invalidObjective = objective;
    for (auto const& [type, search] :
         {std::pair{LineSearchType::Simple, &Ls::Simple},
          std::pair{LineSearchType::Armijo, &Ls::Armijo},
          std::pair{LineSearchType::WolfeWeak, &Ls::WolfeWeak},
          std::pair{LineSearchType::WolfeStrong, &Ls::WolfeStrong},
          std::pair{LineSearchType::ResidualNorm, &Ls::ResidualNorm},
          std::pair{LineSearchType::ArmijoOrResidualNorm, &Ls::ArmijoOrResidualNorm}}) {
      problem.SetSolution(ColumnVector<real>({{1_r, 1_r}}));
      problem.increment.SetConstant(-3_r);
      problem.UpdateObjResDRes({.assemObj = true, .assemRes = true});
      NewtonSolverStatus<real> status;
      status.merit = problem.GetObjective();

      LineSearchParams const params{
          .type = type,
          .maxIter = 2,
          .alpha = 0.5_r,
          .wolfe1 = 1e-4_r,
          .wolfe2 = 0.9_r,
          .maxRelIncrease = 0_r};
      EXPECT_TRUE(search(problem, status, params));
      EXPECT_EQ(status.numLastLSIterDone, 1);
    }
  }
}

/**
 * Two-DoF problem for the step dx = (1, 0) from the zero solution, where the objective is merit0
 * and the residual (startSlope, 0). Elsewhere, the residual is (slope, 1), and the objective is
 * merit0 + |merit0| at the first two trials, dx and dx / 2, and merit0 + rise at the third, dx / 4.
 */
static SnleProblem<real>
GetLineSearchTrialProblem(double merit0, real startSlope, double rise, real slope) {
  auto residual = ColumnVector<real>{2};
  auto assembleFn = [=](SnleProblem<real>& problem, AssemblyParams const& params) mutable {
    real const x = problem.GetSolution()[0];
    if (params.assemObj) {
      problem.objective = (x == 0_r) ? merit0
          : (x == 0.25_r)            ? merit0 + rise
                                     : merit0 + Abs(merit0);
    }
    if (params.assemRes) {
      residual[0] = (x == 0_r) ? startSlope : slope;
      residual[1] = (x == 0_r) ? 0_r : 1_r;
      problem.actorResiduals.resize(1);
      problem.actorResiduals[0] = std::make_pair(0, &residual);
    }
  };

  SnleProblemFunctions<real> functions;
  functions.assemble = std::move(assembleFn);
  functions.onPostNewIncrement = [](SnleProblem<real>& problem) {
    problem.solution += problem.increment;
  };
  int constexpr kNumDofs = 2;
  return SnleProblem<real>{kNumDofs, kNumDofs, std::move(functions)};
}

// A merit-based line search treats a trial that fails its merit test, but changes the merit by no
// more than the band, as passing that test if its slope passes the merit test's Armijo condition;
// the Wolfe searches still apply their curvature test. Out of the band, and for trials that pass
// the merit test, the slope test doesn't apply. The first two trials rise by |merit0|, which every
// merit test and the band reject, so the third, at step scale 1/4, decides, unless the
// residual-norm branch of ArmijoOrResidualNorm accepts earlier.
TEST(NewtonSolver, MeritRoundingErrorBand) {
  using Search = bool (*)(SnleProblem<real>&, NewtonSolverStatus<real>&, LineSearchParams const&);
  std::array<std::pair<LineSearchType, Search>, 5> constexpr kSearches = {
      {{LineSearchType::Simple, &LineSearch<real>::Simple},
       {LineSearchType::Armijo, &LineSearch<real>::Armijo},
       {LineSearchType::WolfeWeak, &LineSearch<real>::WolfeWeak},
       {LineSearchType::WolfeStrong, &LineSearch<real>::WolfeStrong},
       {LineSearchType::ArmijoOrResidualNorm, &LineSearch<real>::ArmijoOrResidualNorm}}};
  real constexpr kEps = std::numeric_limits<real>::epsilon();
  // Rises and slopes are in units of |merit0|, so the band is 2000 kEps. The start slope -kG is
  // small enough that a merit drop of kEps passes every merit test, in both precisions.
  real constexpr kG = 1000_r * kEps;
  double constexpr kNaN = std::numeric_limits<double>::quiet_NaN();
  struct Case {
    double rise{};
    real startSlope{};
    real slope{};
    std::array<bool, 5> accepted{}; // In the order of kSearches
  };
  for (double const merit0 : {4.0, -0.25}) {
    real const unit = Abs(real(merit0));
    for (Case const& c :
         {Case{1500 * kEps, -kG, -0.5_r * kG, {true, true, true, true, true}},
          Case{1500 * kEps, -kG, 1.5_r * kG, {false, false, false, false, false}},
          Case{1500 * kEps, -kG, 0.9999_r * kG, {true, false, false, false, false}}, // c1 = 0
          Case{1500 * kEps, -kG, -0.95_r * kG, {true, true, false, false, true}}, // Wolfe expands
          Case{1500 * kEps, -kG, -kInf, {false, false, false, false, false}},
          Case{1500 * kEps, -kInf, -0.5_r * kG, {false, false, false, false, true}}, // RN passes
          Case{4000 * kEps, -kG, -0.5_r * kG, {false, false, false, false, false}}, // Out of band
          // Out-of-band drop that fails the Armijo test
          Case{-3000 * kEps, -2e8_r * kEps, -4e8_r * kEps, {true, false, false, false, false}},
          // In-band drop that fails the Armijo test
          Case{-10 * kEps, -1e6_r * kEps, -0.5e6_r * kEps, {true, true, true, true, true}},
          Case{-10 * kEps, -kG, 2_r * kG, {true, true, true, false, true}}, // Merit test passes
          Case{kNaN, -kG, -0.5_r * kG, {false, false, false, false, false}}}) { // NaN merit
      std::array<bool, 5> accepted{};
      for (int i = 0; i < isize(kSearches); ++i) {
        auto const [type, search] = kSearches[i];
        SnleProblem<real> problem =
            GetLineSearchTrialProblem(merit0, unit * c.startSlope, unit * c.rise, unit * c.slope);
        problem.SetSolution(ColumnVector<real>::Zero(2));
        problem.UpdateObjResDRes({.assemObj = true, .assemRes = true});
        problem.increment[0] = 1_r;
        problem.increment[1] = 0_r;
        NewtonSolverStatus<real> status;
        status.merit = problem.GetObjective();
        accepted[i] = search(
            problem,
            status,
            {.type = type,
             .maxIter = 3,
             .alpha = 0.5_r,
             .wolfe1 = 1e-4_r,
             .wolfe2 = 0.9_r,
             .maxRelIncrease = 0_r});
      }
      EXPECT_EQ(accepted, c.accepted);
    }
  }
}
