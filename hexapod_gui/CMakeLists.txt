cmake_minimum_required(VERSION 3.0.2)
project(hexapod_gui)

################################
##  CONFIGURE THIS FILE HERE  ##
################################

set(ROS_DEPENDS
  roscpp
  std_msgs
  )

set(EXTERNAL_DEPENDS
  Qt5Widgets
  Qt5PrintSupport
  )

set(EXTERNAL_DEPENDS_LIB
  Qt5::Widgets
  Qt5::PrintSupport
  Qt5::Core
  )

##########################################
##  Be Carefull editing below this line ##
##########################################

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## need this for qt auto generated headers to include!
set(CMAKE_INCLUDE_CURRENT_DIR ON)

###
###  Add Non ROS dependencies
###

foreach(DEPEND ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

###
###  Add any required ros packages here
###
find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDS})

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${ROS_DEPENDS}
  DEPENDS
    ${EXTERNAL_DEPENDS}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRECTORIES}
)

###########
## Build ##
###########

file(GLOB HDRS
  "include/*.h"
)
file(GLOB SRCS
  "src/*.cpp"
  "src/*.hpp"
)
file(GLOB NODES
  "nodes/*.cpp"
)

###
### QT stuff (required if you want a GUI)
###
set(CMAKE_AUTOMOC ON)  #This will autogenerate qt gui stuff
set(CMAKE_AUTORCC ON)  #this needs to be on so cmake will know what to do with QT resource Files

#find_package(Qt5Widgets)
#find_package(Qt5PrintSupport)
#find all the qt UI stuff
file(GLOB_RECURSE UI
    "include/*.ui"
)

message( STATUS "Adding ${PROJECT_NAME} UI files: " )
foreach(source ${UI})
    message( STATUS "  - " ${source})
endforeach(source)

#make them into headers
qt5_wrap_ui (UI_HDRS  ${UI})

message( STATUS "Generating ${PROJECT_NAME} UI Headers: " )
foreach(source ${UI_HDRS})
    message( STATUS "  - " ${source})
endforeach(source)

# find all resource files
file(GLOB QRC
    "rc/*.qrc"
)

###
### compile
###

message( STATUS "buliding ${PROJECT_NAME} node: ")
foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  message( STATUS "  - " ${NODE_NAME})

  add_executable(${NODE_NAME} ${NODE_FILE} ${HDRS} ${SRCS} ${UI} ${QRC})
  add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries (${NODE_NAME} ${catkin_LIBRARIES}  ${EXTERNAL_DEPENDS_LIB})
  target_include_directories(${NODE_NAME} PRIVATE "include/")

  list(APPEND NODE_LIST ${NODE_NAME})

endforeach(NODE_FILE)


install(TARGETS ${NODE_LIST}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
