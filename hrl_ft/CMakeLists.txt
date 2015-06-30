# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(hrl_ft)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rospy tf rosparam visualization_msgs)

catkin_python_setup()

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS rospy tf rosparam visualization_msgs
    INCLUDE_DIRS 
    LIBRARIES 
)

catkin_install_python(PROGRAMS scripts/wrench_zeroing scripts/wrench_to_viz_marker 
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/wrench_zeroing scripts/wrench_to_viz_marker 
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES params/ft_zero_params_gripper.yaml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)
