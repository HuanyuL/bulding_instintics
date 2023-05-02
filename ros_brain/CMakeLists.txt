cmake_minimum_required(VERSION 2.8.3)
project(ros_brain)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


find_library(MULTINEAT MultiNEAT lib)
#(target_variable library_name directory_from_package_root)
#library_name without lib prefix and .so suffix


add_service_files(
        FILES
        TriggerBrain.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        ros_brain
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_brain
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} include)
include_directories(include/MultiNEAT)
include_directories(/home/raimund/ros_workspace/devel/include/)


add_executable(brain src/brain.cpp)
add_executable(brain_server src/brain_server.cpp)
#add_executable(pseudo_sensor src/pseudo_sensor.cpp)

target_link_libraries(brain ${catkin_LIBRARIES})
target_link_libraries(brain_server ${catkin_LIBRARIES})
#target_link_libraries(pseudo_sensor ${catkin_LIBRARIES})
target_link_libraries(brain ${MULTINEAT})
target_link_libraries(brain_server ${MULTINEAT})

install(PROGRAMS
   scripts/mouse_reader.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
#install(TARGETS brain_server
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
#install(TARGETS brain brain_server
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )


## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )


