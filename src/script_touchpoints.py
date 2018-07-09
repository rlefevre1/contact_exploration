#
# Copyright (c) CNRS
# Author: Raphael Lefevre
#

from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, ConstraintGraphFactory, Rule, Constraints
from hpp.gepetto.manipulation import ViewerFactory
from hpp import Transform
from desk_robot import Robot as Desk
from talos_robot import Robot as Talos

import math
import numpy as np
import CORBA

import config_data as coda

# Load robot (talos + desk)
robot = Talos("talos+desk", "talos")
ps = ProblemSolver(robot)
vf = ViewerFactory(ps)

vf.loadObjectModel (Desk, "desk")
v = vf.createViewer()

robot.setJointBounds("talos/root_joint", [-1.5, 1.5, -2, 1, 0.5, 1.5])

q = robot.getCurrentConfig()
pi = 3.141592654
q[0] = 0.3
q[1] = -1.4
q[2] = 0.921
q[5] = math.sin(0.5 * (pi/2))
q[6] = math.cos(0.5 * (pi/2))
# hips offset
q[9] = q[15] = -0.7
# knees offset
q[10] = q[16] = 1.4
# ankles offset
q[11] = q[17] = -0.7
# arms offset
q[24] -= 0.5
q[38] -= 0.5

v(q)

# lock gripper right open
ps.createLockedJoint("gripper_r_lock_idj", "talos/gripper_right_inner_double_joint", [0])
ps.createLockedJoint("gripper_r_lock_f1j", "talos/gripper_right_fingertip_1_joint", [0])
ps.createLockedJoint("gripper_r_lock_f2j", "talos/gripper_right_fingertip_2_joint", [0])
ps.createLockedJoint("gripper_r_lock_isj", "talos/gripper_right_inner_single_joint", [0])
ps.createLockedJoint("gripper_r_lock_f3j", "talos/gripper_right_fingertip_3_joint", [0])
ps.createLockedJoint("gripper_r_lock_j", "talos/gripper_right_joint", [0])
ps.createLockedJoint("gripper_r_lock_msj", "talos/gripper_right_motor_single_joint", [0])
# lock gripper left open
ps.createLockedJoint("gripper_l_lock_idj", "talos/gripper_left_inner_double_joint", [0])
ps.createLockedJoint("gripper_l_lock_f1j", "talos/gripper_left_fingertip_1_joint", [0])
ps.createLockedJoint("gripper_l_lock_f2j", "talos/gripper_left_fingertip_2_joint", [0])
ps.createLockedJoint("gripper_l_lock_isj", "talos/gripper_left_inner_single_joint", [0])
ps.createLockedJoint("gripper_l_lock_f3j", "talos/gripper_left_fingertip_3_joint", [0])
ps.createLockedJoint("gripper_l_lock_j", "talos/gripper_left_joint", [0])
ps.createLockedJoint("gripper_l_lock_msj", "talos/gripper_left_motor_single_joint", [0])
# lock drawer closed
ps.createLockedJoint("drawer_lock", "desk/upper_case_bottom_upper_drawer_bottom_joint", [0])

# static stability constraint
ps.addPartialCom("Talos_CoM", ["talos/root_joint"])
ps.createStaticStabilityConstraints("talos_static_stability", q, "Talos_CoM", ProblemSolver.FIXED_ON_THE_GROUND)
tpc = ps.getPartialCom("Talos_CoM")
rla = robot.getJointPosition(robot.leftAnkle)
com_wf = np.array(tpc)
tf_la = Transform(rla)
com_la = tf_la.inverse().transform(com_wf)
ps.createRelativeComConstraint("talos_com_constraint_to_la", "Talos_CoM", robot.leftAnkle, com_la.tolist(), (True, True, False))

ps.setMaxIterProjection(40) # Set the maximum number of iterations of the projection algorithm. Must be called before the graph creation.

# constraint graph
rules = [
		Rule(["talos/left_gripper", "talos/right_gripper"], ["^$", "^$"], True),
		Rule(["talos/left_gripper", "talos/right_gripper"], ["^$", "desk/touchpoint_right_top"], True),
		Rule(["talos/left_gripper", "talos/right_gripper"], ["^$", "desk/touchpoint_right_front"], True),
		Rule(["talos/left_gripper", "talos/right_gripper"], ["desk/touchpoint_left_front", "^$"], True),
		Rule(["talos/left_gripper", "talos/right_gripper"], ["desk/touchpoint_left_drawer", "^$"], True),
		Rule(["talos/left_gripper", "talos/right_gripper"], ["^$", "desk/upper_drawer_spherical_handle"], True)
		]

graph = ConstraintGraph(robot, "graph")
factory = ConstraintGraphFactory(graph)
factory.setGrippers(["talos/left_gripper", "talos/right_gripper"])
factory.setObjects(["desk"], [["desk/upper_drawer_spherical_handle", "desk/touchpoint_right_top", "desk/touchpoint_right_front", "desk/touchpoint_left_front", "desk/touchpoint_left_drawer"]], [[]])
factory.setRules(rules)
factory.generate()

graph.addConstraints(graph = True,
					 constraints = Constraints(numConstraints = ["talos_com_constraint_to_la", "talos_static_stability/pose-left-foot", "talos_static_stability/pose-right-foot"],
					 lockedJoints = ["gripper_r_lock_idj", "gripper_r_lock_f1j", "gripper_r_lock_f2j", "gripper_r_lock_isj", "gripper_r_lock_f3j", "gripper_r_lock_j", "gripper_r_lock_msj",
									 "gripper_l_lock_idj", "gripper_l_lock_f1j", "gripper_l_lock_f2j", "gripper_l_lock_isj", "gripper_l_lock_f3j", "gripper_l_lock_j", "gripper_l_lock_msj"]))
graph.addConstraints(node = "talos/right_gripper grasps desk/touchpoint_right_top", constraints = Constraints(lockedJoints = ["drawer_lock"]))
graph.addConstraints(node = "talos/right_gripper grasps desk/touchpoint_right_front", constraints = Constraints(lockedJoints = ["drawer_lock"]))
graph.addConstraints(node = "talos/left_gripper grasps desk/touchpoint_left_front", constraints = Constraints(lockedJoints = ["drawer_lock"]))
graph.addConstraints(node = "talos/left_gripper grasps desk/touchpoint_left_drawer", constraints = Constraints(lockedJoints = ["drawer_lock"]))

graph.initialize()

# Remove collision pairs
def removeTouchpointsCollision(gripper, edge, joint, extendToArmEnd = False):
	if gripper == "right":
		graph.removeCollisionPairFromEdge(edge, "talos/gripper_right_fingertip_1_joint", joint)
		graph.removeCollisionPairFromEdge(edge, "talos/gripper_right_fingertip_2_joint", joint)
		graph.removeCollisionPairFromEdge(edge, "talos/gripper_right_fingertip_3_joint", joint)
		graph.removeCollisionPairFromEdge(edge, "talos/gripper_right_inner_single_joint", joint)
		graph.removeCollisionPairFromEdge(edge, "talos/gripper_right_inner_double_joint", joint)
		graph.removeCollisionPairFromEdge(edge, "talos/gripper_right_motor_single_joint", joint)
		graph.removeCollisionPairFromEdge(edge, "talos/gripper_right_joint", joint)
		if extendToArmEnd:
			graph.removeCollisionPairFromEdge(edge, "talos/arm_right_7_joint", joint)
	elif gripper == "left":
		graph.removeCollisionPairFromEdge(edge, "talos/gripper_left_fingertip_1_joint", joint)
		graph.removeCollisionPairFromEdge(edge, "talos/gripper_left_fingertip_2_joint", joint)
		graph.removeCollisionPairFromEdge(edge, "talos/gripper_left_fingertip_3_joint", joint)
		graph.removeCollisionPairFromEdge(edge, "talos/gripper_left_inner_single_joint", joint)
		graph.removeCollisionPairFromEdge(edge, "talos/gripper_left_inner_double_joint", joint)
		graph.removeCollisionPairFromEdge(edge, "talos/gripper_left_motor_single_joint", joint)
		graph.removeCollisionPairFromEdge(edge, "talos/gripper_left_joint", joint)
		if extendToArmEnd:
			graph.removeCollisionPairFromEdge(edge, "talos/arm_left_7_joint", joint)

removeTouchpointsCollision("right", "talos/right_gripper > desk/touchpoint_right_top | f_12", "universe", True)
removeTouchpointsCollision("right", "talos/right_gripper < desk/touchpoint_right_top | 1-1_21" ,"universe", True)
removeTouchpointsCollision("right", "talos/right_gripper > desk/touchpoint_right_front | f_12", "universe")
removeTouchpointsCollision("right", "talos/right_gripper < desk/touchpoint_right_front | 1-2_21", "universe")
removeTouchpointsCollision("left", "talos/left_gripper > desk/touchpoint_left_front | f_12", "universe")
removeTouchpointsCollision("left", "talos/left_gripper < desk/touchpoint_left_front | 0-3_21", "universe")
removeTouchpointsCollision("left", "talos/left_gripper > desk/touchpoint_left_drawer | f_12", "universe")
removeTouchpointsCollision("left", "talos/left_gripper < desk/touchpoint_left_drawer | 0-4_21", "universe")

removeTouchpointsCollision("right", "talos/right_gripper > desk/touchpoint_right_front | f_12", "desk/upper_case_bottom_upper_drawer_bottom_joint")
removeTouchpointsCollision("right", "talos/right_gripper < desk/touchpoint_right_front | 1-2_21", "desk/upper_case_bottom_upper_drawer_bottom_joint")

# create q_init and q_goal respecting the defined constraints
_, q_init, _ = graph.applyNodeConstraints("free", q)
q_goal = q_init[:] ; q_goal[len(q_goal)-1] -= 0.25
_, q_goal, _ = graph.applyNodeConstraints("free", q_goal)
v(q_init)

# create intermediate touchpoints configuration
success3, q_touch_1, residual_error3 = graph.applyNodeConstraints("talos/right_gripper grasps desk/touchpoint_right_top", q_init) # Collision between legs
success4, q_touch_2, residual_error4 = graph.applyNodeConstraints("talos/right_gripper grasps desk/touchpoint_right_front", q_init) # Collision between legs
success1, q_touch_3, residual_error1 = graph.applyNodeConstraints("talos/left_gripper grasps desk/touchpoint_left_front", q_init) # Collision between legs
success2, q_touch_4, residual_error2 = graph.applyNodeConstraints("talos/left_gripper grasps desk/touchpoint_left_drawer", q_init) # fail

# q_touch_1 redefinition
q_touch_1 = coda.q_touch_1

# q_touch_2 redefinition
q_touch_2 = coda.q_touch_2

# q_touch_4 redefinition
q_touch_4 = coda.q_touch_4

ps.addPathOptimizer("Graph-RandomShortcut")

from hpp.gepetto import PathPlayer
pp = PathPlayer(v, ps.client.basic)

def solveAll():
	# trajectory 1
	ps.setInitialConfig(q_init)
	ps.addGoalConfig(q_touch_1)
	graph.setWeight("talos/right_gripper > desk/touchpoint_right_front | f", 0)
	graph.setWeight("talos/left_gripper > desk/touchpoint_left_front | f", 0)
	graph.setWeight("talos/left_gripper > desk/touchpoint_left_drawer | f", 0)
	graph.setWeight("talos/right_gripper > desk/upper_drawer_spherical_handle | f", 0)
	print ps.solve()

	# trajectory 2
	ps.clearRoadmap()
	ps.resetGoalConfigs()
	ps.setInitialConfig(q_touch_1)
	ps.addGoalConfig(q_touch_2)
	graph.setWeight("talos/right_gripper > desk/touchpoint_right_top | f", 0)
	graph.setWeight("talos/right_gripper > desk/touchpoint_right_front | f", 1)
	print ps.solve()

	# trajectory 3
	ps.clearRoadmap()
	ps.resetGoalConfigs()
	ps.setInitialConfig(q_touch_2)
	ps.addGoalConfig(q_touch_3)
	graph.setWeight("talos/right_gripper > desk/touchpoint_right_front | f", 0)
	graph.setWeight("talos/left_gripper > desk/touchpoint_left_front | f", 1)
	print ps.solve()
	
	# trajectory 4
	ps.clearRoadmap()
	ps.resetGoalConfigs()
	ps.setInitialConfig(q_touch_3)
	ps.addGoalConfig(q_touch_4)
	graph.setWeight("talos/left_gripper > desk/touchpoint_left_front | f", 0)
	graph.setWeight("talos/left_gripper > desk/touchpoint_left_drawer | f", 1)
	print ps.solve()

	# end trajectory
	ps.clearRoadmap()
	ps.resetGoalConfigs()
	ps.setInitialConfig(q_touch_4)
	ps.addGoalConfig(q_goal)
	graph.setWeight("talos/left_gripper > desk/touchpoint_left_drawer | f", 0)
	graph.setWeight("talos/right_gripper > desk/upper_drawer_spherical_handle | f", 1)
	print ps.solve()

def play(optimized = True):
	if optimized:
		pp(1); pp(3); pp(5); pp(7); pp(9)
	else:
		pp(0); pp(2); pp(4); pp(6); pp(8)

import time
def visuConfigs(t = 1):
	v(q_init); time.sleep(t)
	v(q_touch_1); time.sleep(t)
	v(q_touch_2); time.sleep(t)
	v(q_touch_3); time.sleep(t)
	v(q_touch_4); time.sleep(t)
	v(q_goal)

#solveAll(); play()