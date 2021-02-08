cmake_minimum_required(VERSION 3.10)
project(autorally_description)
find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
)

install(DIRECTORY urdf/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
   FILES_MATCHING PATTERN "*.dae" PATTERN "*.stl" PATTERN "*.xacro" PATTERN "*.yaml" PATTERN "*.urdf"
)
