cmake_minimum_required(VERSION 2.8.3)
project(hrl_ft)
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

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(FILES params/ft_zero_params_gripper.yaml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)
