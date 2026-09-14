cmake_minimum_required(VERSION 3.8)
project(crane_ocp)

find_package(ament_cmake REQUIRED)

# Builds nothing. The C++ capsule went with issue 135: `crane_mpc`'s `ocp` target
# was its only consumer ever and issue 132 deleted it. What is left is
# `scripts/crane_ocp_export.py`, the acados export boilerplate both exporters share.
#
# `scripts/` is deliberately not installed. Both exporters import it by `sys.path`
# into this source dir, and a source checkout is the precondition of running them
# anyway: they write their own `generated/` and read `crane_model/test/description/`,
# and `crane_ocp_export.py` resolves `crane_model` relative to the caller's source
# dir. Installing this one module removes one source-tree hop of three and costs a
# Python namespace, `ament_cmake_python` and a manifest entry in `crane_planning`,
# which declares none today.

ament_package()
