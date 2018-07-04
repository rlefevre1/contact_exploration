#
# Copyright (c) CNRS
# Author: Raphael Lefevre
#

from hpp.corbaserver.robot import Robot as Parent

class Robot (Parent) :
    rootJointType = "anchor"
    packageName = "hpp_environments"
    urdfName = "drawer_desk/desk"
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__ (self, robotName, load = True):
        Parent.__init__ (self, robotName, self.rootJointType, load)
