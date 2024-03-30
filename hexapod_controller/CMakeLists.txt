cmake_minimum_required(VERSION 3.0.2)
cmake_minimum_required(VERSION 3.0.2)
project( hexapod_controller )

find_package( catkin REQUIRED COMPONENTS nodelet roscpp geometry_msgs sensor_msgs std_msgs hexapod_msgs tf dynamixel_sdk)

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-Wall -Wextra ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hexapod_controller
  LIBRARIES hexapod_controller
  CATKIN_DEPENDS nodelet roscpp geometry_msgs sensor_msgs std_msgs hexapod_msgs tf
)

### FOR PROFILING (need to set after catkin_package)
# http://wiki.ros.org/roslaunch/Tutorials/Profiling%20roslaunch%20nodes
#add_compile_options(-pg)
#set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)
###

include_directories(include ${catkin_INCLUDE_DIRS})

add_library( loop_control src/loop_control.cpp )

add_library( control src/control.cpp )
add_dependencies( control hexapod_msgs_generate_messages_cpp )

add_library( gait src/gait.cpp )
add_dependencies( gait hexapod_msgs_generate_messages_cpp )

add_library( ik src/ik.cpp )
add_dependencies( ik hexapod_msgs_generate_messages_cpp )

add_library( servo_driver src/servo_driver.cpp )
target_link_libraries( servo_driver ${catkin_LIBRARIES} )
add_dependencies( servo_driver hexapod_msgs_generate_messages_cpp )

add_library( parallel_executor src/parallel_executor.cpp )
add_dependencies( parallel_executor servo_driver )

add_executable( hexapod_controller src/hexapod_controller.cpp )
target_link_libraries( hexapod_controller ${catkin_LIBRARIES} control gait ik parallel_executor servo_driver loop_control )
add_dependencies( hexapod_controller hexapod_msgs_generate_messages_cpp )

install(TARGETS hexapod_controller
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES hexapod_controller.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

