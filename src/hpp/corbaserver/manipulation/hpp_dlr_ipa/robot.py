#!/usr/bin/env python
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

from hpp.corbaserver.manipulation.robot import Robot as Parent

##
#  Control of robot DLR IPA in hpp
#
#  This class implements a client to the corba server implemented in
#  hpp-corbaserver. It derive from class hpp.corbaserver.robot.Robot.
#
#  This class is also used to initialize a client to gepetto-viewer in order to
#  display configurations of the robot.
#
#  At creation of an instance, the urdf and srdf files are loaded using
#  idl interface hpp::corbaserver::Robot::loadRobotModel.
class Robot (Parent):
    ##
    #  Information to retrieve urdf and srdf files.
    packageName = "hpp-dlr-ipa"
    urdfName = "ipa325_ur10_screwdriver"
    urdfSuffix = ""
    srdfSuffix = ""

    ## Constructor
    # \param compositeName name of the composite robot that will be built later,
    # \param robotName name of the first robot that is loaded now,
    # \param load whether to actually load urdf files. Set to no if you only
    #        want to initialize a corba client to an already initialized
    #        problem.
    # \param rootJointType type of root joint among ("freeflyer", "planar",
    #        "anchor"),
    def __init__ (self, compositeName, robotName, load = True,
                  rootJointType = "anchor"):
        Parent.__init__ (self, compositeName, robotName, rootJointType, load)

