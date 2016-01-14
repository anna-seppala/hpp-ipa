# Copyright (c) 2015 CNRS
# Author: Anna Seppala
#
# This file is part of hpp-dlr-ipa.
# hpp-dlr-ipa is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-dlr-ipa is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-dlr-ipa.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME hpp-dlr-ipa)
SET(PROJECT_DESCRIPTION "Data specific to dlr ipa robot")
SET(PROJECT_URL "")

SETUP_PROJECT()

# ADD_REQUIRED_DEPENDENCY (ur_description)
ADD_REQUIRED_DEPENDENCY (xacro)

# Copy xacro files to ROS package
SET (XACROS
urdf/door.urdf.xacro
urdf/ipa325_ur10.urdf.xacro
urdf/ipa325_ur10_empty.urdf.xacro
urdf/ipa325_ur10_screwdriver.urdf.xacro
urdf/platform.urdf.xacro
urdf/tool_changer.urdf.xacro
urdf/tool_gripper.urdf.xacro
urdf/tool_screwdriver.urdf.xacro
)

install (FILES
  ${XACROS}
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/urdf )


SET (XACRO_EXEC ${XACRO_PREFIX}/share/xacro/xacro.py)
MESSAGE (STATUS "XACRO_EXEC=${XACRO_EXEC}")

execute_process(
  COMMAND ${XACRO_EXEC} -o ipa325_ur10_empty.urdf
  ipa325_ur10_empty.urdf.xacro
 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/urdf
  )

execute_process(
  COMMAND ${XACRO_EXEC} -o ipa325_ur10_gripper.urdf
  ipa325_ur10_gripper.urdf.xacro
 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/urdf
  )
execute_process(
  COMMAND ${XACRO_EXEC} -o ipa325_ur10_screwdriver.urdf
  ipa325_ur10_screwdriver.urdf.xacro
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/urdf
 )


execute_process(
  COMMAND ${XACRO_EXEC} -o door.urdf
  door.urdf.xacro
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/urdf
 )

 message("current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")
 message("project source dir: ${PROJECT_SOURCE_DIR}")
 message("cmake install data root dir: ${CMAKE_INSTALL_DATAROOTDIR}")
INSTALL(FILES
  urdf/door.urdf
  urdf/ipa325_ur10_empty.urdf
  urdf/ipa325_ur10_gripper.urdf
  urdf/ipa325_ur10_screwdriver.urdf
DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/hpp-dlr-ipa/urdf
)
INSTALL(FILES
  srdf/door.srdf
  srdf/ipa325_ur10_empty.srdf
  srdf/ipa325_ur10_gripper.srdf
  srdf/ipa325_ur10_screwdriver.srdf
 DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/hpp-dlr-ipa/srdf
)

INSTALL (FILES
  package.xml
 DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/hpp-dlr-ipa
)

# Copy mesh files to ROS package
SET (MESHES
  meshes/Door_m.STL
  meshes/Mobile_Platform_Shell_m.STL
  meshes/Tool_Gripper_Shell_m.STL
  meshes/Tools_Screwdriver_Shell_m.STL
  meshes/Tools_to_Tool_Changer_m.STL
)

install (FILES
  ${MESHES}
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/hpp-dlr-ipa/meshes )

 message("install data root dir: ${CMAKE_INSTALL_DATAROOTDIR}")
 message("catkin pkg share destination: ${CATKIN_PACKAGE_SHARE_DESTINATION}")
# message("PYTHON_PACKAGES_DIR: ${PYTHON_PACKAGES_DIR}")
ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
