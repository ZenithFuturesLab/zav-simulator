cmake_minimum_required(VERSION 3.16)
project(epic_ros2 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
set(ROS_DEPS rclcpp rosgraph_msgs std_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs
  builtin_interfaces message_filters pcl_conversions tf2 tf2_geometry_msgs cv_bridge)
foreach(dep ${ROS_DEPS})
  find_package(${dep} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree segmentation)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolyTraj.msg" "msg/PositionCommand.msg" "msg/TakeoffLand.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces)
rosidl_get_typesupport_target(message_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
ament_export_dependencies(rosidl_default_runtime)

set(G ${CMAKE_CURRENT_SOURCE_DIR}/src/global_planner)
set(L ${CMAKE_CURRENT_SOURCE_DIR}/src/local_planner)
set(MODULES
  ${G}/lidar_map ${G}/pointcloud_topo ${G}/frontier_manager
  ${G}/exploration_manager ${G}/utils/lkh_tsp_solver
  ${L}/path_searching ${L}/minco_planner ${L}/Utils/poly_traj ${L}/Utils/traj_utils)
set(EPIC_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
foreach(module ${MODULES})
  list(APPEND EPIC_INCLUDES ${module}/include)
endforeach()

file(GLOB LKH_SOURCES CONFIGURE_DEPENDS ${G}/utils/lkh_tsp_solver/src/*.c)
add_library(epic_lkh SHARED ${LKH_SOURCES} ${G}/utils/lkh_tsp_solver/src/lkh_interface.cpp)
target_include_directories(epic_lkh PRIVATE ${G}/utils/lkh_tsp_solver/include)
target_compile_options(epic_lkh PRIVATE $<$<COMPILE_LANGUAGE:C>:-fcommon>)
target_link_libraries(epic_lkh m)

add_library(epic_core STATIC
  ${G}/lidar_map/src/ikd_Tree.cpp ${G}/lidar_map/src/sim_lio.cpp ${G}/lidar_map/src/lidar_map_interface.cpp
  ${G}/pointcloud_topo/src/topo_skeleton_graph.cpp ${G}/pointcloud_topo/src/historical_odom_graph.cpp
  ${G}/pointcloud_topo/src/bubble_split.cpp ${G}/pointcloud_topo/src/parallel_bubble_astar.cpp
  ${G}/frontier_manager/src/frontier_manager.cpp ${G}/frontier_manager/src/global_planning.cpp
  ${G}/frontier_manager/src/perimeter_survey.cpp
  ${G}/frontier_manager/src/raycast.cpp ${G}/frontier_manager/src/frontier_viszer.cpp
  ${L}/path_searching/src/bubble_astar.cpp ${L}/Utils/poly_traj/src/polynomial_traj.cpp
  ${L}/Utils/traj_utils/src/planning_visualization.cpp
  ${L}/minco_planner/src/planner_manager.cpp ${G}/exploration_manager/src/fast_exploration_manager.cpp)
target_include_directories(epic_core PUBLIC ${EPIC_INCLUDES})
target_compile_options(epic_core PUBLIC -include epic_ros2/compat.hpp)
ament_target_dependencies(epic_core ${ROS_DEPS})
target_link_libraries(epic_core Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX epic_lkh "${message_typesupport}")

add_executable(exploration_node ${G}/exploration_manager/src/exploration_node.cpp
  ${G}/exploration_manager/src/fast_exploration_fsm.cpp ${G}/exploration_manager/src/fsm_utils.cpp)
target_link_libraries(exploration_node epic_core)
add_executable(traj_server ${L}/minco_planner/src/traj_server.cpp)
target_link_libraries(traj_server epic_core)
set_target_properties(exploration_node traj_server PROPERTIES INSTALL_RPATH "$ORIGIN/..")

install(TARGETS epic_lkh LIBRARY DESTINATION lib)
install(TARGETS exploration_node traj_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config docs DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE README.md DESTINATION share/${PROJECT_NAME})
install(FILES scripts/epic_frames.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/isaac_adapter.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  add_executable(gcopter_quadrature_test test/gcopter_quadrature_test.cpp)
  target_include_directories(gcopter_quadrature_test PRIVATE ${L}/minco_planner/include)
  target_link_libraries(gcopter_quadrature_test Eigen3::Eigen)
  target_compile_options(gcopter_quadrature_test PRIVATE -UNDEBUG -O0)
  add_test(NAME gcopter_quadrature_test COMMAND gcopter_quadrature_test)
  add_executable(goal_handoff_test test/goal_handoff_test.cpp)
  target_link_libraries(goal_handoff_test epic_core)
  # The native library remains optimized; avoid optimizing the assertion driver.
  target_compile_options(goal_handoff_test PRIVATE -UNDEBUG -O0)
  add_test(NAME goal_handoff_test COMMAND goal_handoff_test --ros-args
    --params-file ${CMAKE_CURRENT_SOURCE_DIR}/config/isaac_warehouse_cruise.yaml)
  set_tests_properties(goal_handoff_test PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=97;ROS_LOCALHOST_ONLY=1;CYCLONEDDS_URI=")
  add_executable(frontier_clustering_test test/frontier_clustering_test.cpp)
  target_link_libraries(frontier_clustering_test epic_core)
  target_compile_options(frontier_clustering_test PRIVATE -UNDEBUG -O0)
  add_test(NAME frontier_clustering_test COMMAND frontier_clustering_test --ros-args
    --params-file ${CMAKE_CURRENT_SOURCE_DIR}/config/isaac_warehouse_cruise.yaml)
  set_tests_properties(frontier_clustering_test PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=97;ROS_LOCALHOST_ONLY=1;CYCLONEDDS_URI=")
  add_executable(surface_visibility_test test/surface_visibility_test.cpp)
  target_link_libraries(surface_visibility_test epic_core)
  target_compile_options(surface_visibility_test PRIVATE -UNDEBUG -O0)
  add_test(NAME surface_visibility_test COMMAND surface_visibility_test --ros-args
    --params-file ${CMAKE_CURRENT_SOURCE_DIR}/config/isaac_building_explore.yaml)
  set_tests_properties(surface_visibility_test PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=97;ROS_LOCALHOST_ONLY=1;CYCLONEDDS_URI=")
endif()
ament_package()
