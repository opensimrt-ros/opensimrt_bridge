cmake_minimum_required(VERSION 3.0.2)
project(opensimrt_bridge)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

add_compile_options(-std=c++17 -O3)

find_package(catkin REQUIRED COMPONENTS
  opensimrt
  opensimrt_msgs
  roscpp
  tf2
  tf2_ros
)

find_package(OpenSim REQUIRED)

catkin_package(
  CATKIN_DEPENDS opensimrt_msgs roscpp
  tf2
  tf2_ros
  INCLUDE_DIRS include
  LIBRARIES conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_library(osb ) ##maybe make it header only and avoid this
message(STATUS "opensimrt libraries := ${opensimrt_LIBRARIES}")

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

add_executable(saver src/ik_saver.cpp)

add_dependencies(saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(saver
  ${catkin_LIBRARIES}
  ${OpenSim_LIBRARIES}
  ${opensimrt_LIBRARIES}
)

add_executable(grf_dump src/grf_mot_file_dumper.cpp)

add_dependencies(grf_dump ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(grf_dump
	conversions
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
file (GLOB conversions_SOURCES CONFIGURE_DEPENDS "src/conversions/*.cpp")
file (GLOB conversions_HEADERS CONFIGURE_DEPENDS "include/opensimrt_bridge/conversions/*.h")

set(target conversions)

add_library(conversions ${conversions_HEADERS} ${conversions_SOURCES} )

target_link_libraries(conversions ${catkin_LIBRARIES} ${OpenSim_LIBRARIES})

add_dependencies(conversions ${catkin_EXPORTED_TARGETS})

