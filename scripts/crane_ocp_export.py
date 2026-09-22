"""
The acados export boilerplate, once.

`crane_mpc/scripts/export_ocp.py` and `crane_planning/scripts/export_timing_ocp.py`
define two different problems -- a tracking OCP over time and a
minimum-traversal-time OCP over sigma -- and used to open with the same forty
lines: the same `sys.path` hop into `crane_model/scripts`, the same scratch-JSON
code generation, the same pruning of what acados writes and this stack does not
ship, the same whitespace normalisation, the same `--check` byte-compare, and the
same `z` code-generated beside the solver.

That part is here. **Nothing that knows a problem is**: no cost, no constraint, no
row order, no dimension. Each exporter still writes its own `build_ocp` and its
own generated header, which is the whole of what makes them two exporters.

It is imported by path rather than installed, exactly as both exporters already
import `crane_symbolic`. Deliberately -- `CMakeLists.txt` records why::

    OCP_PACKAGE = PACKAGE.parent / "crane_ocp"
    sys.path.insert(0, str(OCP_PACKAGE / "scripts"))
    import crane_ocp_export as ox
"""

from __future__ import annotations

import argparse
import filecmp
import shutil
import sys
import tempfile
from pathlib import Path

import casadi as ca
import yaml
from acados_template import AcadosOcpSolver

# What acados generates and this stack does not ship. `main_*.c` carry a `main`,
# so compiling them into a library is not merely wasteful; the sim solver is a
# second artifact nothing here uses; the `Makefile` is the only generated file
# that carries an absolute path, and `acados_solver.pxd` is for Cython.
#
# `_hess.c` is the exact Hessian of the residual, which
# `generate_c_code_nls_cost` writes unconditionally -- there is no option for it,
# unlike the dynamics, whose `generate_hess` follows `hessian_approx`. Under
# `GAUSS_NEWTON` nothing registers it: it appears in neither
# `acados_solver_<name>.c` nor `<name>_cost.h`, only in the pruned `Makefile`. It
# is also the largest thing acados writes for these problems, so shipping it
# would carry megabytes of dead code.
UNSHIPPED = ("Makefile", "acados_solver.pxd")
UNSHIPPED_PREFIXES = ("main_", "acados_sim_solver_")
UNSHIPPED_SUFFIXES = ("_hess.c",)


def import_crane_symbolic(package: Path):
    """
    Import the shared symbolic model, `crane_model.symbolic`.

    It lives in the installed Python package because `crane_planning` builds its
    OCP from it at runtime. An export run before `crane_model` is on the path --
    a from-scratch build of a sibling package -- falls back to the source tree.
    Returns the module; the caller binds it to whatever name it uses.
    """
    try:
        from crane_model import symbolic
    except ImportError:
        sys.path.insert(0, str(package.parent / "crane_model"))
        from crane_model import symbolic

    return symbolic


def default_descriptions(package: Path) -> Path:
    """Where the two expanded machine descriptions are, from a sibling package."""
    return package.parent / "crane_model" / "test" / "description"


def read_ros_parameters(path: Path, node: str) -> dict:
    """Read one ROS parameter file down to a node's `ros__parameters`."""
    with open(path) as stream:
        root = yaml.safe_load(stream)
    parameters = root[node]["ros__parameters"]
    if not isinstance(parameters, dict):
        raise ValueError(f"{path}: {node}.ros__parameters is not a mapping")
    return parameters


def write_output_map(model, name: str, tree: Path) -> None:
    """
    Code-generate the shared output map beside a solver, in physical units.

    acados generates only what it solves, and what it solves is rows already
    divided by their conditioning constants. `wiki/mpc.md` §5.3 requirement 4
    wants the residuals themselves, `OcpNode` wants what the answer demands of the
    machine, and the planner's bisected warm start wants to know how far outside
    its bounds a node is -- so `tau_a`, `F_cyl`, `v` and `Q` are shipped as one
    more generated function, all six axes of each.

    Six and not five on purpose: the tool cylinder is still in the model when its
    coordinate leaves the OCP, and reporting what it carries is how "the
    transmission did not leave with the coordinate" stays checkable from outside.

    It is `crane_symbolic`'s own `z` over the module's own `(x, u, p)`, so the two
    packages ship **the same function** up to the symbol prefix.
    """
    function = ca.Function(
        f"{name}_output",
        [model.x, model.u, model.p],
        [ca.densify(model.z)],
        ["x", "u", "p"],
        ["z"],
    )
    generator = ca.CodeGenerator(
        f"{name}_output.c",
        {
            "mex": False,
            "casadi_int": "int",
            "casadi_real": "double",
            "with_header": True,
        },
    )
    generator.add(function)
    generator.generate(str(tree) + "/")


def normalise(path: Path) -> None:
    """Strip trailing whitespace and leave exactly one final newline."""
    text = path.read_text()
    body = "\n".join(line.rstrip() for line in text.splitlines())
    path.write_text(body.rstrip("\n") + "\n")


def prune(tree: Path) -> None:
    """Delete what acados generates and this stack does not ship."""
    for path in sorted(tree.rglob("*")):
        if not path.is_file():
            continue
        if (
            path.name in UNSHIPPED
            or path.name.startswith(UNSHIPPED_PREFIXES)
            or path.name.endswith(UNSHIPPED_SUFFIXES)
        ):
            path.unlink()


def generate_solver(ocp, output: Path) -> Path:
    """
    Code-generate one assembled `AcadosOcp` into `output/<model name>/`, pruned.

    The JSON is the only artifact that carries the absolute path of the tree it
    was written into, so it goes to a scratch file rather than into the
    repository: a checked-in copy would differ between a devcontainer and a native
    checkout for a reason that has nothing to do with the OCP.
    """
    name = ocp.model.name
    tree = output / name
    ocp.code_export_directory = str(tree)
    with tempfile.TemporaryDirectory() as scratch:
        AcadosOcpSolver.generate(ocp, json_file=str(Path(scratch) / f"{name}.json"))
    prune(tree)
    return tree


def finalise(output: Path, readme: str) -> None:
    """
    Write the tree's README and make the whole tree a fixed point of the hooks.

    `ament_cpplint` and `ament_cppcheck` match `.c` and `.h`, and machine output
    fails them by thousands; acados' file names are fixed, so the tree cannot dodge
    them by extension. What it can be is a fixed point of the hooks that *rewrite*
    files -- `trailing-whitespace`, `end-of-file-fixer` and `mixed-line-ending`
    then have nothing to change and cannot silently break `--check`. Do not pass a
    `generated/` tree to `pre-commit run --files`.
    """
    (output / "README.md").write_text(readme)
    for path in sorted(output.rglob("*")):
        if path.is_file():
            normalise(path)


def compare(left: Path, right: Path) -> list:
    """Return every path under `left` that `right` does not match byte for byte."""
    differences = []
    names = {path.relative_to(left) for path in left.rglob("*") if path.is_file()}
    names |= {path.relative_to(right) for path in right.rglob("*") if path.is_file()}
    for name in sorted(names):
        one = left / name
        other = right / name
        if not one.is_file() or not other.is_file():
            differences.append(name)
        elif not filecmp.cmp(one, other, shallow=False):
            differences.append(name)
    return differences


def add_arguments(parser: argparse.ArgumentParser, package: Path) -> None:
    """Add the three options every exporter in this stack takes."""
    parser.add_argument(
        "--descriptions",
        type=Path,
        default=default_descriptions(package),
        help="where the two expanded machine descriptions are",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=package / "generated",
        help="where the generated solvers go",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="regenerate into a scratch tree and diff instead of rewriting",
    )


def run(output: Path, check: bool, generate, script: str, clean=True, note="") -> int:
    """
    Rewrite `output`, or regenerate into a scratch tree and diff it.

    `generate` takes the directory to write into and writes the whole tree from
    empty, which is what makes `--check` a comparison of two trees produced the
    same way rather than of a tree against a partial rewrite of itself.
    """
    if not check:
        if clean and output.exists():
            shutil.rmtree(output)
        generate(output)
        print(f"wrote {output}{note}")
        return 0

    with tempfile.TemporaryDirectory() as scratch:
        fresh = Path(scratch) / "generated"
        generate(fresh)
        differences = compare(output, fresh)
    if differences:
        print(
            f"the checked-in solver is not what `{script}` writes today. "
            "Re-run the script and commit the result; if only the CasADi or "
            "acados banner moved, the toolchain changed rather than the model."
        )
        for name in differences:
            print(f"  {name}")
        return 1
    print("the checked-in solver is current")
    return 0
