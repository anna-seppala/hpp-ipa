from math import sqrt, pi
from hpp.corbaserver.manipulation.hpp_dlr_ipa import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph
from hpp.gepetto.manipulation import ViewerFactory
from hpp.gepetto import PathPlayer

## create class that describes door object and how to load it
class Door (object):
   rootJointType = 'anchor'
   packageName = 'hpp-dlr-ipa'
   meshPackageName = 'hpp-dlr-ipa'
   urdfName = 'door'
   urdfSuffix = ""
   srdfSuffix = ""

## create class for box object (uncomment if
## introducing obstacles)
#class Box (object):
#  rootJointType = 'anchor'
#  packageName = 'hpp_tutorial'
#  meshPackageName = 'hpp_tutorial'
#  urdfName = 'box'
#  urdfSuffix = ""
#  srdfSuffix = ""

robot = Robot ('dlr-box', 'dlr')
## root joint position change for robot:
robot.client.manipulation.robot.setRootJointPosition('dlr', [0,0,0,1,0,0,0])

## create problem solver and choose path optimiser
## to view the selection of available optimisers, call:
## ps.client.basic.problem.getAvailable('PathOptimizer')
## Optimisers with the prefix 'Graph' may be used in manipulation tasks
ps = ProblemSolver (robot)
ps.addPathOptimizer ('Graph-RandomShortcut')
## to undo path optimiser selection, call ps.clearPathOptimizers () 

## create viewerFactory. Useful if there is no need to
## visualise the problem until later (done by calling r.createRealClient ())
## alternatively, import 'Viewer' from hpp.gepetto to visualise from the beginning
r = ViewerFactory (ps)

## load door model into the viewer
r.loadObjectModel (Door, 'door')
robot.client.manipulation.robot.setRootJointPosition('door', [0.5,1.6,0.8,1,0,0,0])
## create an extra degree of freedom to execute tasks that seemignly have the same
## initial and goal configuration (which is generally not supported by the path planner)
## by changing the value of the extra DoF, the successful execution of the task is registered
robot.client.basic.robot.setDimensionExtraConfigSpace (4)
robot.client.basic.robot.setExtraConfigSpaceBounds ([0,1, 0,1, 0,1, 0,1])

## load box-shaped objects as obstacles into the problem.
## uncomment the following code and the class definition 'class Box' from before.
#r.loadObjectModel (Box, 'box1')
#robot.client.manipulation.robot.setRootJointPosition('box1', [0.3,1,0.9,1,0,0,0])
#r.loadObjectModel (Box, 'box2')
#robot.client.manipulation.robot.setRootJointPosition('box2', [0.6,1.05,1.0,1,0,0,0])
#r.loadObjectModel (Box, 'box3')
#robot.client.manipulation.robot.setRootJointPosition('box3', [1.1,1,0.9,1,0,0,0])
#r.loadObjectModel (Box, 'box4')
#robot.client.manipulation.robot.setRootJointPosition('box4', [0.9,1.05,1.2,1,0,0,0])

## create the constraint graph
graph = ConstraintGraph (robot, 'graph')

## Create lists of joint names - useful to define passive joints (not used in this tutorial).
jointNames = dict ()
jointNames['all'] = robot.getJointNames ()
jointNames['dlr'] = list ()
for n in jointNames['all']:
  if n.startswith ("dlr"):
    jointNames['dlr'].append (n)
    ps.client.manipulation.problem.createLockedJoint (n, n, [0])

lockRobot = jointNames['all'][:]
robotPassiveDof = 'robotPassiveDof'
ps.addPassiveDofs (robotPassiveDof, jointNames['dlr'])

## give initial configuration (note extraDoFs that extend config size)
q_init = [0, -pi/2, 0, -pi/2, 0, 0, 0, 0, 0, 0]
r(q_init)

## create grasps with name, associate gripper and wanted handle to grasp
## create locked extraDof -> Dof must be locked when not in contact with 'door' in order
## to detect successful execution of task 
graph.createGrasp ('first', 'dlr/screwdriver', 'door/door_lower_right_corner')
graph.createPreGrasp ('firstPre', 'dlr/screwdriver', 'door/door_lower_right_corner')
ps.client.manipulation.problem.createLockedExtraDof ('extra1', 0, [0])

graph.createGrasp ('second', 'dlr/screwdriver', 'door/window_lower_left_corner')
graph.createPreGrasp ('secondPre', 'dlr/screwdriver', 'door/window_lower_left_corner')
ps.client.manipulation.problem.createLockedExtraDof ('extra2', 1, [0])

graph.createGrasp ('third', 'dlr/screwdriver', 'door/window_lower_right_corner')
graph.createPreGrasp ('thirdPre', 'dlr/screwdriver', 'door/window_lower_right_corner')
ps.client.manipulation.problem.createLockedExtraDof ('extra3', 2, [0])

graph.createGrasp ('fourth', 'dlr/screwdriver', 'door/door_handle')
graph.createPreGrasp ('fourthPre', 'dlr/screwdriver', 'door/door_handle')
ps.client.manipulation.problem.createLockedExtraDof ('extra4', 3, [0])

## create nodes for contact with door and free end-effector
## the order of nodes is essential: first comes the most restrictive node, 
## followed by those less so.
graph.createNode (['door1','door2','door3','door4','free1',])

## create waypoint edges to make reaching the goal state easier
## type help(graph.createWaypointEdge) for more details 
graph.createWaypointEdge ('free1', 'door1', "grasp1", nb=1, weight=1)
graph.createWaypointEdge ('free1', 'door2', "grasp2", nb=1, weight=1)
graph.createWaypointEdge ('free1', 'door3', "grasp3", nb=1, weight=1)
graph.createWaypointEdge ('free1', 'door4', "grasp4", nb=1, weight=1)

## create edges for movement when gripper is free and when in contact
graph.createEdge ('free1', 'free1', 'move_free1', 0)
graph.createLevelSetEdge ('door1', 'door1', 'keep_grasp1', 5)
graph.createLevelSetEdge ('door2', 'door2', 'keep_grasp2', 5)
graph.createLevelSetEdge ('door3', 'door3', 'keep_grasp3', 5)
graph.createLevelSetEdge ('door4', 'door4', 'keep_grasp4', 5)

## Set the constraints of the component of the graph.
## when moving without touching the desired point (moving to or from the 'door1' node),
## the extra DoFs have to be locked (change of value impeded)
## additionally, the extra DoFs that should not go through a hange of value must be locked
## when in contact as well.
## thus, only one extra DoF registers a change when the desired point is reached 
graph.setConstraints (edge='move_free1', lockDof=['extra1', 'extra2', 'extra3', 'extra4'])

graph.setConstraints (node='door1', grasps = ['first',])
graph.setConstraints (edge='keep_grasp1', lockDof=['extra2', 'extra3', 'extra4']+lockRobot)
graph.setLevelSetConstraints (edge='keep_grasp1', lockDof=['extra1',])

graph.setConstraints (node="grasp1_n0", pregrasps = ['firstPre',])
graph.setConstraints (edge='grasp1_e0',  lockDof=['extra1','extra2', 'extra3', 'extra4'])
graph.setConstraints (edge='grasp1_e1',  lockDof=['extra1','extra2', 'extra3', 'extra4'])
graph.client.graph.setShort (graph.edges["grasp1_e1"], True)

graph.setConstraints (node='door2', grasps = ['second',])
graph.setConstraints (edge='keep_grasp2', lockDof=['extra1', 'extra3', 'extra4']+lockRobot)
graph.setLevelSetConstraints (edge='keep_grasp2', lockDof=['extra2',])

graph.setConstraints (node="grasp2_n0", pregrasps = ['secondPre',])
graph.setConstraints (edge='grasp2_e0', lockDof=['extra1', 'extra2', 'extra3', 'extra4'])
graph.setConstraints (edge='grasp2_e1', lockDof=['extra1', 'extra2', 'extra3', 'extra4'])
graph.client.graph.setShort (graph.edges["grasp2_e1"], True)

graph.setConstraints (node='door3', grasps = ['third',])
graph.setConstraints (edge='keep_grasp3', lockDof=['extra1', 'extra2', 'extra4']+lockRobot)
graph.setLevelSetConstraints (edge='keep_grasp3', lockDof=['extra3',])

graph.setConstraints (node="grasp3_n0", pregrasps = ['thirdPre',])
graph.setConstraints (edge='grasp3_e0', lockDof=['extra1', 'extra2', 'extra3', 'extra4'])
graph.setConstraints (edge='grasp3_e1', lockDof=['extra1', 'extra2', 'extra3', 'extra4'])
graph.client.graph.setShort (graph.edges["grasp3_e1"], True)

graph.setConstraints (node='door4', grasps = ['fourth',])
graph.setConstraints (edge='keep_grasp4', lockDof=['extra1', 'extra2', 'extra3']+lockRobot)
graph.setLevelSetConstraints (edge='keep_grasp4', lockDof=['extra4',])

graph.setConstraints (node="grasp4_n0", pregrasps = ['fourthPre',])
graph.setConstraints (edge='grasp4_e0', lockDof=['extra1', 'extra2', 'extra3', 'extra4'])
graph.setConstraints (edge='grasp4_e1', lockDof=['extra1', 'extra2', 'extra3', 'extra4'])
graph.client.graph.setShort (graph.edges["grasp4_e1"], True)

## to visualise the created graph in a browser, call
## graph.display(format='svg')

## Initial and Goal configurations must differ from each other, the same
## initial and goal configuration (which is generally not supported by the path planner)
q_goal = q_init [::]
q_goal [6] = 1
ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)

## solve for first contact. The solve function adds computed path into a
## path vector. If path optimisers are used, multiple paths are added
## into the vector (unoptimised, optimised with 1st optimiser, optimised with 2nd optimiser etc.)
ps.solve ()

## to solve for second contact, reset goal configurations and
## set a new one
q_goal = q_init[::]
q_goal [7] = 1
ps.resetGoalConfigs()
ps.addGoalConfig (q_goal)

## 2nd contact
ps.solve()

q_goal = q_init[::]
q_goal [8] = 1
ps.resetGoalConfigs()
ps.addGoalConfig (q_goal)

## 3rd contact
ps.solve()

q_goal = q_init[::]
q_goal [9] = 1
ps.resetGoalConfigs()
ps.addGoalConfig (q_goal)

## 4th contact
ps.solve()

## create real viewer from factory to visualise computed paths
v = r.createRealClient ()
## create a path player. To play paths, use the index of computed path in path vector
pp = PathPlayer (robot.client.basic, v)
pp (0) # visualise first task without optimisation
pp (1) #   		-..-	with optimisation 
pp (3) # 2nd optimised
pp (5) # 3rd optimised
pp (7) # 4th optimised

## use in separate window to interrupt ps.solve() :
## from hpp.corbaserver import Client
## client = Client()
## client.problem.interruptPathPlanning()
