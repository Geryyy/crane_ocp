cmake_minimum_required(VERSION 3.8)
project(crane_ocp)

find_package(ament_cmake REQUIRED)
# acados, and this is the one package in the stack that names it in a public
# header. `wiki/implementation/libraries.md` §1 lists it for "the optimal control
# problems of `mpc` and `trajectory_planning`", so no SQP loop, no condensing and
# no integrator is written here -- only the binding.
#
# **How CI resolves it.** The image installs acados to `/usr/local` and its config
# lands at `/usr/local/cmake/acadosConfig.cmake`, which `<prefix>/cmake/` on the
# config-mode search path finds without a hint. There is no rosdep key, so
# `package.xml`'s `<depend>acados</depend>` is documentation and the image is what
# provides it. Same resolution `crane_mpc` and `crane_planning` already record.
find_package(acados REQUIRED)

# STATIC and position-independent, for the reason every other library in this
# stack records: the consumers are static archives that end up inside executables
# and, through `crane_control`, inside a pluginlib shared object.
add_library(${PROJECT_NAME} STATIC src/acados_solver.cpp)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
# PUBLIC, unlike everywhere else acados appears: `acados_solver.hpp` names
# `ocp_nlp_config` and its neighbours, because the whole point of this package is
# that the binding is written once. A consumer of *this* header is a consumer of
# acados by construction; nothing downstream of `Ocp` or `TimingOcp` is, and both
# of those keep acados PRIVATE exactly as they did.
target_link_libraries(${PROJECT_NAME} PUBLIC acados)

install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(
  EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(acados)

ament_package()
