"""
The acados export, once, for every OCP in this stack.

`crane_mpc/scripts/export_ocp.py` and `crane_planning/scripts/export_timing_ocp.py`
define two different problems -- a tracking OCP over time and a
minimum-traversal-time OCP over sigma. Everything that is not the problem is
here: the `sys.path` hop into `crane_model/scripts`, the parameter read, and the
export itself.

**Nothing that knows a problem is**: no cost, no constraint, no row order, no
dimension.

Imported by path rather than installed, exactly as both exporters already import
`crane_symbolic`::

    OCP_PACKAGE = PACKAGE.parent / "crane_ocp"
    sys.path.insert(0, str(OCP_PACKAGE / "scripts"))
    import crane_ocp_export as ox

## One path, where there were three

This module used to code-generate into a reviewable `generated/` tree that
nothing compiled, while `crane_mpc.solver.load_or_build` and
`crane_planning.ocp`'s `cache_tree` each generated *and* compiled the same
problem again into their own hashed cache, at startup. Three copies of the same
boilerplate, and the tree a reviewer read was not the artifact either node ran.

Now `export` generates and compiles, `open_solver` opens, and the trade is
deliberate: **the export is a step somebody runs.** A startup no longer compiles,
so it can no longer compile the wrong thing quietly -- it opens what is there and
refuses if that is not this problem.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path

import yaml
from acados_template import AcadosOcpSolver, AcadosSimSolver

#: Written beside the compiled solver; carries the key it was exported under.
MANIFEST = "manifest.json"

#: acados' own json, and the directory it compiles into.
OCP_JSON = "ocp.json"
OCP_CODE = "c_generated_code"


class StaleExport(RuntimeError):
    """No exported solver, or one built from a different problem."""


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


def export_base(env: str, leaf: str) -> Path:
    """
    Where exports are written and read: `$env`, else a persistent user cache.

    Not the system temporary directory, which is where the per-startup caches
    lived: an export somebody runs deliberately has to survive a tmp sweep, or
    the next run refuses instead of starting.
    """
    root = os.environ.get(env)
    return (Path(root) if root else Path.home() / ".cache" / "crane_ocp") / leaf


def key_digest(key: dict) -> str:
    return hashlib.sha256(
        json.dumps(key, sort_keys=True, default=str).encode()
    ).hexdigest()[:10]


def solver_root(base: Path, key: dict) -> Path:
    """
    One directory per problem, named for it.

    Not a cache -- nothing here ever builds on demand. It is how several
    exports coexist, which they have to: `crane_planning` gets its description
    off `/robot_description` and sim and hardware are different xacro, and a
    sweep walks one configuration per variant.
    """
    return base / key_digest(key)


def add_arguments(parser: argparse.ArgumentParser, package: Path, env: str, leaf: str):
    """Add the options every exporter in this stack takes."""
    parser.add_argument(
        "--descriptions",
        type=Path,
        default=default_descriptions(package),
        help="where the two expanded machine descriptions are",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=export_base(env, leaf),
        help="where the compiled solver goes; the runtime reads the same place",
    )
    parser.add_argument("--verbose", action="store_true", help="acados' own output")


def installed_acados(target) -> None:
    """
    Point acados_template at the installed headers and libraries.

    Use `/usr/local` only if `lib/link_libs.json` is present; else fall to
    `ACADOS_SOURCE_DIR`.
    """
    prefix = Path("/usr/local")
    if (
        (prefix / "include" / "acados").is_dir()
        and (prefix / "lib" / "libacados.so").is_file()
        and (prefix / "lib" / "link_libs.json").is_file()
    ):
        # acados 0.5.4 moved both onto `code_gen_options`; AcadosSim's are read-only.
        options = getattr(target, "code_gen_options", target)
        options.acados_include_path = str(prefix / "include")
        options.acados_lib_path = str(prefix / "lib")


def divergence(wanted: dict, carried: dict) -> list:
    """
    Name every field the export disagrees with, not just that one does.

    A bare hash mismatch is not enough to act on when re-exporting costs
    minutes: this reads as "the horizon moved" rather than "reason unknown".
    """
    reasons = []
    for name in sorted(set(wanted) | set(carried)):
        one, other = wanted.get(name, "<absent>"), carried.get(name, "<absent>")
        if one != other:
            reasons.append(f"  {name}: asked for {one!r}, exported {other!r}")
    return reasons


def export(ocp, base: Path, key: dict, *, sims=(), verbose: bool = False) -> Path:
    """
    Generate **and compile** one problem into `root`, recording `key` beside it.

    `sims` is `(label, AcadosSim)` pairs -- integrators over the same model that
    a runtime needs, exported here so they are opened rather than built too.

    Nothing is pruned. The tree path used to delete `_hess.c` because it shipped
    its output into git and compiled none of it; here the compile is the point,
    and that pruning was a `GAUSS_NEWTON` assumption which made
    `hessian_approx: EXACT` link against a function nobody had built.
    """
    root = solver_root(base, key)
    root.mkdir(parents=True, exist_ok=True)
    installed_acados(ocp)
    ocp.code_export_directory = str(root / OCP_CODE)
    AcadosOcpSolver(
        ocp, json_file=str(root / OCP_JSON), generate=True, build=True, verbose=verbose
    )
    for label, sim in sims:
        installed_acados(sim)
        sim.code_export_directory = str(root / f"sim_{label}_code")
        AcadosSimSolver(
            sim,
            json_file=str(root / f"sim_{label}.json"),
            generate=True,
            build=True,
            verbose=verbose,
        )
    (root / MANIFEST).write_text(
        json.dumps(
            {"key": key, "sims": [label for label, _ in sims]},
            indent=2,
            sort_keys=True,
            default=str,
        )
        + "\n"
    )
    return root


def manifest(base: Path, key: dict, command: str) -> Path:
    """
    Resolve this problem's export and return it; refuse when there is none.

    The refusal names what every *other* export on disk differs in, because the
    usual cause is one exported for another machine or another setting, and
    "differs in description" is actionable where "not found" is not.
    """
    root = solver_root(base, key)
    if (root / MANIFEST).is_file():
        return root
    others = []
    for path in sorted(base.glob(f"*/{MANIFEST}")):
        try:
            carried = json.loads(path.read_text()).get("key", {})
        except (OSError, ValueError):
            continue
        differ = [
            name
            for name in set(key) | set(carried)
            if key.get(name) != carried.get(name)
        ]
        others.append(f"  {path.parent.name}: differs in {', '.join(sorted(differ))}")
    raise StaleExport(
        "\n".join(
            [f"no exported solver for this problem in {base}"]
            + (others or ["  nothing else is exported there either"])
            + [f"export one with `{command}`"]
        )
    )


def open_solver(ocp, base: Path, key: dict, command: str, *, verbose: bool = False):
    """Open the exported solver, with the dims acados wrote beside it."""
    root = manifest(base, key, command)
    installed_acados(ocp)
    ocp.code_export_directory = str(root / OCP_CODE)
    json_path = root / OCP_JSON
    solver = AcadosOcpSolver(
        ocp, json_file=str(json_path), generate=False, build=False, verbose=verbose
    )
    return solver, json.loads(json_path.read_text())["dims"]


def open_sim(sim, base: Path, key: dict, label: str, *, verbose: bool = False):
    """Open one exported integrator. `open_solver` resolved the same root."""
    root = solver_root(base, key)
    installed_acados(sim)
    sim.code_export_directory = str(root / f"sim_{label}_code")
    return AcadosSimSolver(
        sim,
        json_file=str(root / f"sim_{label}.json"),
        generate=False,
        build=False,
        check_reuse_possible=False,
        verbose=verbose,
    )
