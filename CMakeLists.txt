cmake_minimum_required(VERSION 2.8.3)
project(cmvision)

find_package(catkin REQUIRED 
  COMPONENTS 
  cv_bridge
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  Blob.msg
  Blobs.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories(${wxWidgets_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(src/cmvision)

## Declare a cpp library
add_library(${PROJECT_NAME} STATIC
    src/cmvision/cmvision.h
    src/cmvision/cmvision.cc
    src/cmvision/capture.h
    src/cmvision/conversions.h
    src/cmvision/conversions.c
)
add_dependencies(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# declare our main node
add_executable(${PROJECT_NAME}_node
	src/nodes/main.cpp
    src/nodes/cmvision_color_blob_finder.h
    src/nodes/cmvision_color_blob_finder.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_executable(colorgui
    src/nodes/color_gui.h
    src/nodes/color_gui.cpp
)
target_link_libraries(colorgui
    fltk
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${wxWidgets_LIBRARIES}
)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node colorgui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

