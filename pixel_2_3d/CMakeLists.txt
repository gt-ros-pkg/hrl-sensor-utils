cmake_minimum_required(VERSION 2.8.3)
project(pixel_2_3d)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation geometry_msgs tf pcl_ros image_transport image_geometry)

find_package(PCL REQUIRED)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(pixel_2_3d src/pixel_2_3d.cpp)

target_link_libraries(pixel_2_3d ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_service_files(
  DIRECTORY srv
  FILES Pixel23d.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
    DEPENDS roscpp geometry_msgs image_transport tf image_geometry message_runtime pcl_ros
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

