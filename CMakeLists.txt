cmake_minimum_required(VERSION 3.8)
project(crane_ocp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Builds nothing. The C++ capsule went with issue 135: `crane_mpc`'s `ocp` target
# was its only consumer ever and issue 132 deleted it. What is left is
# `crane_ocp_export`, the one acados export path in this stack.
#
# It *is* installed now, which reverses the note that used to sit here. That note
# was right while this module only exported: both exporters imported it by
# `sys.path` into this source dir, and a source checkout is the precondition of
# running an exporter anyway. It stopped being right when the module took over
# opening the compiled solver as well -- `crane_mpc` and `crane_planning` both
# import it at *runtime* now, and an installed node has no source tree to hop
# into.
ament_python_install_package(crane_ocp_export)

ament_package()
