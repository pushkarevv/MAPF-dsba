#pragma once

#include "solvers/solver_base.h"

#include "models/models.h"

namespace mapf::solver {

struct AStarSolver : public SolverBase {
    models::MAPFSolution FindSolution(const models::MAPFProblem& mapf_problem) const final;
};

}  // namespace mapf::solver

