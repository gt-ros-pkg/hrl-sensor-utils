cmake_minimum_required(VERSION 2.8.3)
project(hrl_clickable_display)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs cv_bridge image_transport tf image_geometry std_srvs visualization_msgs message_generation)

find_package( OpenCV REQUIRED )

add_message_files(
  DIRECTORY msg
  FILES ArrowOverlayCmd.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS roscpp geometry_msgs cv_bridge image_transport opencv2 tf image_geometry std_srvs visualization_msgs message_runtime
    CATKIN_DEPENDS
    INCLUDE_DIRS 
    LIBRARIES 
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(clickable_display src/clickable_display.cpp)
add_executable(arrow_overlay src/arrow_overlay.cpp)

target_link_libraries(clickable_display ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(arrow_overlay ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS clickable_display arrow_overlay
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
