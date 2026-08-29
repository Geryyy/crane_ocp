#include "crane_ocp/acados_solver.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace crane_ocp
{

std::string status_word(int status)
{
  switch (status) {
    case ACADOS_SUCCESS: return "ACADOS_SUCCESS";
    case ACADOS_NAN_DETECTED: return "ACADOS_NAN_DETECTED";
    case ACADOS_MAXITER: return "ACADOS_MAXITER";
    case ACADOS_MINSTEP: return "ACADOS_MINSTEP";
    case ACADOS_QP_FAILURE: return "ACADOS_QP_FAILURE";
    case ACADOS_READY: return "ACADOS_READY";
    case ACADOS_TIMEOUT: return "ACADOS_TIMEOUT";
    default: return "acados status " + std::to_string(status);
  }
}

bool evaluate(
  AcadosFunction function, std::vector<const double *> arguments, double * result)
{
  double * results[1] = {result};
  return function(arguments.data(), results, nullptr, nullptr, nullptr) == 0;
}

bool evaluate(
  CasadiFunction function, std::vector<const double *> arguments, double * result)
{
  double * results[1] = {result};
  return function(arguments.data(), results, nullptr, nullptr, 0) == 0;
}

AcadosSolver::~AcadosSolver()
{
  if (capsule_ != nullptr) {
    backend_.destroy(capsule_);
    backend_.free_capsule(capsule_);
  }
}

int AcadosSolver::open(int intervals, const std::vector<double> & steps)
{
  capsule_ = backend_.create_capsule();
  if (capsule_ == nullptr) {
    return kCapsuleUnavailable;
  }
  std::vector<double> grid = steps;
  const int created = backend_.create_with_grid(capsule_, intervals, grid.data());
  if (created != ACADOS_SUCCESS) {
    return created;
  }
  intervals_ = intervals;
  config_ = backend_.config(capsule_);
  dims_ = backend_.dims(capsule_);
  in_ = backend_.in(capsule_);
  out_ = backend_.out(capsule_);
  solver_ = backend_.solver(capsule_);
  opts_ = backend_.opts(capsule_);
  return created;
}

int AcadosSolver::open(int intervals, double step)
{
  return open(intervals, std::vector<double>(static_cast<std::size_t>(intervals), step));
}

bool AcadosSolver::set_parameters(int stage, double * values, int count)
{
  return backend_.update_params(capsule_, stage, values, count) == 0;
}

void AcadosSolver::set_cost(int stage, const char * field, double * values)
{
  ocp_nlp_cost_model_set(config_, dims_, in_, stage, field, values);
}

void AcadosSolver::set_constraint(int stage, const char * field, double * values)
{
  ocp_nlp_constraints_model_set(config_, dims_, in_, stage, field, values);
}

void AcadosSolver::set_option(const char * field, void * value)
{
  ocp_nlp_solver_opts_set(config_, opts_, field, value);
}

void AcadosSolver::set_iterate(int stage, const char * field, double * values)
{
  ocp_nlp_out_set(config_, dims_, out_, stage, field, values);
}

void AcadosSolver::get_iterate(int stage, const char * field, double * values) const
{
  ocp_nlp_out_get(config_, dims_, out_, stage, field, values);
}

void AcadosSolver::get_statistic(const char * field, void * value) const
{
  ocp_nlp_get(solver_, field, value);
}

void AcadosSolver::reset()
{
  ocp_nlp_out_set_values_to_zero(config_, dims_, out_);
  ocp_nlp_solver_reset_qp_memory(solver_, in_, out_);
}

int AcadosSolver::solve()
{
  return backend_.solve(capsule_);
}

}  // namespace crane_ocp
