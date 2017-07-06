cmake_minimum_required(VERSION 2.8.3)
project(robor_utilities)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS tf joy rviz roscpp roslaunch nodelet std_srvs nav_msgs sensor_msgs geometry_msgs)
find_package(Armadillo REQUIRED)
find_package(Boost 1.54.0 REQUIRED system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES state_estimator simulator reference_generator data_recorder youbot_visualizer optitrack_drift
  CATKIN_DEPENDS tf joy rviz roscpp roslaunch nodelet std_srvs nav_msgs sensor_msgs geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS})

#
# Build libs
#
add_library(state_estimator src/state_estimator.cpp)
target_link_libraries(state_estimator ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
add_dependencies(state_estimator ${catkin_EXPORTED_TARGETS})

add_library(simulator src/simulator.cpp)
target_link_libraries(simulator ${catkin_LIBRARIES})
add_dependencies(simulator ${catkin_EXPORTED_TARGETS})

add_library(reference_generator src/reference_generator.cpp)
target_link_libraries(reference_generator ${catkin_LIBRARIES})
add_dependencies(reference_generator ${catkin_EXPORTED_TARGETS})

add_library(data_recorder src/data_recorder.cpp)
target_link_libraries(data_recorder ${catkin_LIBRARIES})
add_dependencies(data_recorder ${catkin_EXPORTED_TARGETS})

add_library(youbot_visualizer src/youbot_visualizer.cpp)
target_link_libraries(youbot_visualizer ${catkin_LIBRARIES})
add_dependencies(youbot_visualizer ${catkin_EXPORTED_TARGETS})

add_library(optitrack_drift src/optitrack_drift.cpp)
target_link_libraries(optitrack_drift ${catkin_LIBRARIES})
add_dependencies(optitrack_drift ${catkin_EXPORTED_TARGETS})

#
# Build nodes
#
add_executable(state_estimator_node src/nodes/state_estimator_node.cpp)
target_link_libraries(state_estimator_node state_estimator)

add_executable(simulator_node src/nodes/simulator_node.cpp)
target_link_libraries(simulator_node simulator)

add_executable(reference_generator_node src/nodes/reference_generator_node.cpp)
target_link_libraries(reference_generator_node reference_generator)

add_executable(data_recorder_node src/nodes/data_recorder_node.cpp)
target_link_libraries(data_recorder_node data_recorder)

add_executable(youbot_visualizer_node src/nodes/youbot_visualizer_node.cpp)
target_link_libraries(youbot_visualizer_node youbot_visualizer)

add_executable(optitrack_drift_node src/nodes/optitrack_drift_node.cpp)
target_link_libraries(optitrack_drift_node optitrack_drift)

#
# Build nodelets
#
add_library(${PROJECT_NAME}_nodelets
  src/nodelets/state_estimator_nodelet.cpp
  src/nodelets/simulator_nodelet.cpp
  src/nodelets/reference_generator_nodelet.cpp
  src/nodelets/data_recorder_nodelet.cpp
  src/nodelets/youbot_visualizer_nodelet.cpp
  src/nodelets/optitrack_drift_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelets state_estimator simulator reference_generator data_recorder youbot_visualizer optitrack_drift)

#
# Build rviz plugins
#
set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME}_gui
    src/displays/path_display.cpp include/${PROJECT_NAME}/displays/path_display.h
    src/displays/pose_visual.cpp include/${PROJECT_NAME}/displays/pose_visual.h
    #
    src/panels/state_estimator_panel.cpp include/${PROJECT_NAME}/panels/state_estimator_panel.h
    src/panels/simulator_panel.cpp include/${PROJECT_NAME}/panels/simulator_panel.h
    src/panels/reference_generator_panel.cpp include/${PROJECT_NAME}/panels/reference_generator_panel.h
    src/panels/data_recorder_panel.cpp include/${PROJECT_NAME}/panels/data_recorder_panel.h)
target_link_libraries(${PROJECT_NAME}_gui ${roscpp_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(${PROJECT_NAME}_gui ${catkin_EXPORTED_TARGETS})

#
# Install libraries
#
install(TARGETS state_estimator simulator reference_generator data_recorder youbot_visualizer optitrack_drift ${PROJECT_NAME}_nodelets ${PROJECT_NAME}_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#
# Install nodes
#
install(TARGETS state_estimator_node simulator_node reference_generator_node data_recorder_node youbot_visualizer_node optitrack_drift_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#
# Install header files
#
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#
# Install nodelet and rviz plugins description
#
install(FILES nodelet_plugins.xml rviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#
# Install launch files
#
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#
# Create folders and copy resources
#
file(MAKE_DIRECTORY $ENV{HOME}/.local/share/icons/robor)
file(COPY resources/play.png resources/pause.png resources/stop.png DESTINATION $ENV{HOME}/.local/share/icons/robor)
