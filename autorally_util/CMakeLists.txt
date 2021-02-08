cmake_minimum_required(VERSION 3.10)
project(autorally_util)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package()

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
   FILES_MATCHING PATTERN "*"
)
