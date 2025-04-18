# Newer CMake features (like target_link_libraries with imported targets) need ≥ 3.0
cmake_minimum_required(VERSION 3.0.2)
project(uav_gazebo)
add_compile_options(-std=c++17)

# pulls in ROS‑specific macros, include paths, libraries
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  std_msgs
  geometry_msgs
  nav_msgs
  roscpp
  angles
  tf2_ros
  uav_gazebo_msgs
  dynamic_reconfigure
  sensor_msgs
  trajectory_msgs
  kdl_parser
)

generate_dynamic_reconfigure_options(
  cfg/PurePursuitController.cfg
)

# looks for a system‑ or workspace‑installed Ruckig/KDL/Gazebo package
find_package(ruckig REQUIRED) 
find_package(orocos_kdl REQUIRED)
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Exports include paths and libraries to downstream packages
# If someone else builds a package depending on this, they’ll automatically see these ROS deps
catkin_package(CATKIN_DEPENDS 
                roscpp 
                std_msgs 
                geometry_msgs 
                nav_msgs 
                tf2_ros 
                uav_gazebo_msgs
                sensor_msgs
                trajectory_msgs
              #  DEPENDS ruckig
)

# Compiler instructions where to find headers
include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${GAZEBO_INCLUDE_DIRS}
                    ${ruckig_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_plugin SHARED src/${PROJECT_NAME}_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} -Wl,--no-undefined)

add_executable(simple_drone_manager src/simple_drone_manager.cpp)
target_link_libraries(simple_drone_manager ${catkin_LIBRARIES})

add_executable(example_ruckig_position src/example_ruckig_position.cpp)
target_link_libraries(example_ruckig_position 
                      ${catkin_LIBRARIES} 
                      ${orocos_kdl_LIBRARIES}
                      ruckig::ruckig)

install(
  TARGETS ${PROJECT_NAME}_plugin simple_drone_manager example_ruckig_position
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install your Python node
catkin_install_python(PROGRAMS
  script/example_ruckig_position.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)