#!/usr/bin/env python
import math
import random
import sys
import pybullet as p
from time import sleep

# mode = 'ik'
mode = 'nullspace'

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
jointsMap = []
framesMap = []
t = 0
dt = 0.01
p.setPhysicsEngineParameter(fixedTimeStep=dt)

# Collecting the available joints
for k in range (nJoints):
    jointInfo = p.getJointInfo(robot, k)
    name = jointInfo[1].decode('utf-8')
    print('~> '+name)
    if '_fixing' not in name:
        if '_frame' in name:
            framesMap.append([name, k])
        else:
            jointsMap.append(k)

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
    
    for name, joint in framesMap:
        if name == 'tip_frame':
            orn = p.getQuaternionFromEuler([0, 1.2, 0])
            pos = [0.18, 0.02, 0.48+0.05*math.sin(t*3)]

            if mode == 'ik':
                jointPoses = p.calculateInverseKinematics(
                    robot, joint, pos, maxNumIterations=1000)

            if mode == 'nullspace':
                jointPoses = p.calculateInverseKinematics(
                    robot, joint, pos, lowerLimits=ll, upperLimits=ul, jointRanges=jr, restPoses=rp, jointDamping=jd, maxNumIterations=100)

            print(jointPoses)
            jointPoses = list(jointPoses)

            for i in range(len(jointsMap)):
                p.resetJointState(robot, jointsMap[i], jointPoses[i])

    print('~')
    for name, joint in framesMap:
        if name == 'tip_frame':
            print('Frame '+name)
            jointState = p.getLinkState(robot, joint)
            pos = jointState[0]
            orientation = p.getEulerFromQuaternion(jointState[1])
            print(pos)
            print(orientation)

            if line is not None:
                p.addUserDebugLine(line, pos, [1,0,1], 2, 10)
            line = pos
        
    p.stepSimulation()
    sleep(dt)
