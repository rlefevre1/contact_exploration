#
# Copyright (c) CNRS
# Author: Raphael Lefevre
#

from hpp.corbaserver.manipulation.robot import Robot as Parent

class Robot (Parent) :
    rootJointType = "freeflyer"
    packageName = "talos-data"
    urdfName = "talos"
    urdfSuffix = "_full_collision"
    srdfSuffix = ""

    def __init__ (self, compositeName, robotName, load = True):
        Parent.__init__ (self, compositeName, robotName, self.rootJointType, load)
        self.rightAnkle = robotName + "/leg_right_6_joint"
        self.leftAnkle = robotName + "/leg_left_6_joint"
        self.rightWrist = robotName + "/arm_right_7_joint"
        self.leftWrist = robotName + "/arm_left_7_joint"
