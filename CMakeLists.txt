cmake_minimum_required(VERSION 3.0.2)
project(opensimrt_bridge)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  opensimrt
  opensimrt_msgs
  roscpp
)

find_package(OpenSim REQUIRED)

catkin_package(
  CATKIN_DEPENDS opensimrt_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ik_dump src/ik_file_dumper.cpp)

add_dependencies(ik_dump ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ik_dump
  ${catkin_LIBRARIES}
  ${OpenSim_LIBRARIES}
  ${opensimrt_LIBRARIES}
)

add_executable(id_dump src/id_file_dumper.cpp)

add_dependencies(id_dump ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(id_dump
  ${catkin_LIBRARIES}
  ${OpenSim_LIBRARIES}
  ${opensimrt_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/bridge.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenSim_LIBRARIES}
  ${opensimrt_LIBRARIES}
)

