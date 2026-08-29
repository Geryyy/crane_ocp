// The acados layer, once.
//
// `crane_mpc` and `crane_planning` each ship a generated acados solver and each
// used to carry its own forty capsule calls to open it, write what stays
// runtime-settable, solve, and read the answers back. The two problems are
// different -- a tracking OCP over time against a minimum-traversal-time OCP over
// sigma -- but *that* part of them was the same code twice, down to the macro
// that writes the per-tool thunks and the switch that turns an acados status
// into a word.
//
// It is its own package rather than part of `crane_model` because `crane_control`
// depends on the model and has no reason to compile against acados.
//
// What is **not** here: anything that knows a problem. No dimensions, no row
// order, no cost, no constraint. Those stay in the exporter that baked them and
// in the binding that reads them off its own generated header.

#ifndef CRANE_OCP__ACADOS_SOLVER_HPP_
#define CRANE_OCP__ACADOS_SOLVER_HPP_

#include <string>
#include <vector>

extern "C" {
#include "acados_c/ocp_nlp_interface.h"
}

namespace crane_ocp
{

/// acados' own status word, as the message a refusal carries.
[[nodiscard]] std::string status_word(int status);

/// The two shapes of generated function this stack calls.
/**
 * acados' own generated entry points take a `void *` memory handle; CasADi's
 * `CodeGenerator` output takes an `int` memory token. Both are otherwise the
 * same signature, and both are called with one output block.
 */
using AcadosFunction = int (*)(const double **, double **, int *, double *, void *);
using CasadiFunction = int (*)(const double **, double **, int *, double *, int);

/// Evaluate a generated function into `result`. False is a backend failure.
[[nodiscard]] bool evaluate(
  AcadosFunction function, std::vector<const double *> arguments, double * result);
[[nodiscard]] bool evaluate(
  CasadiFunction function, std::vector<const double *> arguments, double * result);

/// One generated solver, as the entry points acados writes beside it.
/**
 * The capsule type differs per artifact -- that is what makes two generated
 * solvers two solvers rather than one parameterised one -- so the handle is
 * `void *` and `CRANE_OCP_ACADOS_BACKEND` is where the cast lives, in exactly one
 * place per entry point per artifact.
 */
struct AcadosBackend
{
  const char * name{nullptr};
  void * (*create_capsule)() = nullptr;
  int (*create_with_grid)(void *, int, double *) = nullptr;
  int (*solve)(void *) = nullptr;
  int (*destroy)(void *) = nullptr;
  int (*free_capsule)(void *) = nullptr;
  int (*update_params)(void *, int, double *, int) = nullptr;
  ocp_nlp_config * (*config)(void *) = nullptr;
  ocp_nlp_dims * (*dims)(void *) = nullptr;
  ocp_nlp_in * (*in)(void *) = nullptr;
  ocp_nlp_out * (*out)(void *) = nullptr;
  ocp_nlp_solver * (*solver)(void *) = nullptr;
  void * (*opts)(void *) = nullptr;
};

// `NAME` is the acados model name the exporter chose, which is the prefix every
// generated symbol carries. One macro, four uses in `crane_mpc` and two in
// `crane_planning`; the alternative is thirty lines of identical thunks with a
// different token pasted into every symbol, which is what a macro is for.
#define CRANE_OCP_ACADOS_BACKEND(NAME)                                                 \
  []() {                                                                               \
    using Capsule = NAME ## _solver_capsule;                                           \
    crane_ocp::AcadosBackend backend;                                                  \
    backend.name = #NAME;                                                              \
    backend.create_capsule = []() -> void * {return NAME ## _acados_create_capsule();}; \
    backend.create_with_grid = [](void * h, int n, double * steps) {                   \
        return NAME ## _acados_create_with_discretization(static_cast<Capsule *>(h), n, steps); \
      };                                                                               \
    backend.solve = [](void * h) {return NAME ## _acados_solve(static_cast<Capsule *>(h));};   \
    backend.destroy = [](void * h) {return NAME ## _acados_free(static_cast<Capsule *>(h));};  \
    backend.free_capsule =                                                             \
      [](void * h) {return NAME ## _acados_free_capsule(static_cast<Capsule *>(h));};  \
    backend.update_params = [](void * h, int stage, double * value, int np) {          \
        return NAME ## _acados_update_params(static_cast<Capsule *>(h), stage, value, np); \
      };                                                                               \
    backend.config =                                                                   \
      [](void * h) {return NAME ## _acados_get_nlp_config(static_cast<Capsule *>(h));}; \
    backend.dims =                                                                     \
      [](void * h) {return NAME ## _acados_get_nlp_dims(static_cast<Capsule *>(h));};  \
    backend.in = [](void * h) {return NAME ## _acados_get_nlp_in(static_cast<Capsule *>(h));}; \
    backend.out = [](void * h) {return NAME ## _acados_get_nlp_out(static_cast<Capsule *>(h));}; \
    backend.solver =                                                                   \
      [](void * h) {return NAME ## _acados_get_nlp_solver(static_cast<Capsule *>(h));}; \
    backend.opts =                                                                     \
      [](void * h) {return NAME ## _acados_get_nlp_opts(static_cast<Capsule *>(h));};  \
    return backend;                                                                    \
  }()

/// Every acados object one solve owns, freed in the order acados wants.
class AcadosSolver
{
public:
  /// What `open` returns when the capsule itself would not allocate.
  static constexpr int kCapsuleUnavailable = -1;

  explicit AcadosSolver(AcadosBackend backend)
  : backend_(backend) {}

  ~AcadosSolver();

  AcadosSolver(const AcadosSolver &) = delete;
  AcadosSolver & operator=(const AcadosSolver &) = delete;
  AcadosSolver(AcadosSolver &&) = delete;
  AcadosSolver & operator=(AcadosSolver &&) = delete;

  /// Open the shipped solver on this solve's own grid; acados' create status.
  /**
   * `<name>_acados_create_with_discretization(capsule, N, steps)` is generated
   * beside the fixed-`N` entry point, so the interval count and the step widths
   * are arguments and the artifact's own grid is a default rather than a
   * contract.
   */
  int open(int intervals, const std::vector<double> & steps);

  /// The same, on a uniform grid of `intervals` steps of `step`.
  int open(int intervals, double step);

  [[nodiscard]] bool ready() const noexcept {return capsule_ != nullptr;}
  [[nodiscard]] const char * name() const noexcept {return backend_.name;}
  [[nodiscard]] int intervals() const noexcept {return intervals_;}

  /// Write `p` onto one stage. False is the solver refusing the vector.
  [[nodiscard]] bool set_parameters(int stage, double * values, int count);

  void set_cost(int stage, const char * field, double * values);
  void set_constraint(int stage, const char * field, double * values);
  void set_option(const char * field, void * value);

  /// Write one block of the initial iterate -- `x`, `u`, `sl`, `su`, `lam`, `pi`.
  void set_iterate(int stage, const char * field, double * values);

  /// Read one block of the solution back.
  void get_iterate(int stage, const char * field, double * values) const;

  /// Read one of the solver's own statistics -- `sqp_iter`, `time_tot`, ...
  void get_statistic(const char * field, void * value) const;

  /// Zero the iterate and drop the QP memory, so a solve starts where it is told.
  void reset();

  [[nodiscard]] int solve();

private:
  AcadosBackend backend_;
  void * capsule_{nullptr};
  int intervals_{0};
  ocp_nlp_config * config_{nullptr};
  ocp_nlp_dims * dims_{nullptr};
  ocp_nlp_in * in_{nullptr};
  ocp_nlp_out * out_{nullptr};
  ocp_nlp_solver * solver_{nullptr};
  void * opts_{nullptr};
};

}  // namespace crane_ocp

#endif  // CRANE_OCP__ACADOS_SOLVER_HPP_
