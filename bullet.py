#!/usr/bin/env python
import math
import random
import sys
import pybullet as p
from time import sleep

mode = 'ik'
# mode = 'nullspace'

directory = 'quadruped/'

# Instanciation de Bullet
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -10)

# Chargement du sol
planeId = p.loadURDF('bullet/plane.urdf')

# Chargement du robot
cubeStartPos = [0, 0, 0.5]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robot = p.loadURDF(directory+"/robot.urdf",
                       cubeStartPos, cubeStartOrientation)
cubePos, cubeOrn = p.getBasePositionAndOrientation(robot)
nJoints = p.getNumJoints(robot)

# Map des joints
tip_frame = None
joints = []
t = 0
dt = 0.01
p.setPhysicsEngineParameter(fixedTimeStep=dt)

# Collecting the available joints
for k in range (nJoints):
    jointInfo = p.getJointInfo(robot, k)
    name = jointInfo[1].decode('utf-8')
    if '_fixing' not in name:
        if '_frame' not in name:
            joints.append(k)
        elif name == 'tip_frame':
            tip_frame = k

# lower limits for null space
ll=[-3]*12
# upper limits for null space
ul=[3]*12
# joint ranges for null space
jr=[0]*12
# restposes for null space
rp=[0]*12
# joint damping coefficents
jd=[0.1]*12

# Preventing leg to bend in wrong direction in null-space mode
ll[8] = 0

line = None

while True:
    t += dt
    
    # Traget position
    pos = [0.18, 0.02, 0.48+0.05*math.sin(t*3)]

    if mode == 'ik':
        jointPoses = p.calculateInverseKinematics(
            robot, tip_frame, pos, maxNumIterations=1000)

    if mode == 'nullspace':
        jointPoses = p.calculateInverseKinematics(
            robot, tip_frame, pos, lowerLimits=ll, upperLimits=ul, jointRanges=jr, restPoses=rp, jointDamping=jd, maxNumIterations=100)

    for i in range(len(joints)):
        p.resetJointState(robot, joints[i], jointPoses[i])

    print('~')
    print('Tip position:')
    jointState = p.getLinkState(robot, tip_frame)
    pos = jointState[0]
    print(pos)

    if line is not None:
        p.addUserDebugLine(line, pos, [1,0,1], 2, 10)
    line = pos
        
    p.stepSimulation()
    sleep(dt)
