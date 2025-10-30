import pybullet as p
import matplotlib.pyplot as plt
import numpy as np
import time
import math
import pybullet_data
physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.setPhysicsEngineParameter(numSolverIterations=200)
p.setRealTimeSimulation(0)
p.setTimeStep(1/240)
planeId = p.loadURDF("plane.urdf")
startPos = [-0.5, 0, 0.5]
startOrientation = p.getQuaternionFromEuler([0, 0, 3.14159])
robotId = p.loadURDF("/Users/liranz/Downloads/testing.urdf",
                     startPos, startOrientation)
p.resetDebugVisualizerCamera(
    cameraDistance=4,
    cameraYaw=0,
    cameraPitch=-10,
    cameraTargetPosition=[0.4, 0, 0.5]
)
p.resetBasePositionAndOrientation(robotId, startPos, startOrientation)
num_joints = p.getNumJoints(robotId)
step_width = 1  # all length units are in meters
step_height = 0.15
step_depth = 0.23
num_steps = 9
stairs = []
for i in range(num_steps):
    x = i * step_depth
    z = (i+0.5) * step_height
    collision = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[step_depth/2, step_width/2, step_height/2])
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[
                                 step_depth/2, step_width/2, step_height/2], rgbaColor=[0.5, 0.5, 0.5, 1])
    stairId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision,
                                baseVisualShapeIndex=visual, basePosition=[x, 0, z])
    stairs.append(stairId)
for i in range(num_joints):
    print(i, p.getJointInfo(robotId, i)[1])
left_state = p.getLinkState(robotId, 0)
right_state = p.getLinkState(robotId, 1)
stair_friction = 1  # friction coefficient between wheel and stairs
p.changeDynamics(planeId, -1, lateralFriction=1)
p.changeDynamics(robotId, -1, lateralFriction=0.3)
for stair in stairs:
    p.changeDynamics(stair, -1, lateralFriction=stair_friction)
for i in range(num_joints):
    p.changeDynamics(robotId, i, lateralFriction=stair_friction)
for i in range(240):
    p.stepSimulation()
    time.sleep(1/240)
driving_torque = 6.3
p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=0,
                        controlMode=p.VELOCITY_CONTROL, targetVelocity=3, force=driving_torque)
p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=1,
                        controlMode=p.VELOCITY_CONTROL, targetVelocity=3, force=driving_torque)
climb_start = 0
# used to keep track of how many simulation steps passed since beginning of trial
climb_attempts = 0
climb_successes = 0
speedup_factor = 300  # factor to which simulation is sped up by
sim_steps = 0
while True:
    left_state = p.getLinkState(robotId, 0)  # left wheel info
    right_state = p.getLinkState(robotId, 1)  # right wheel info
    climb_end = sim_steps
    pos, orn = p.getBasePositionAndOrientation(robotId)
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)
    if yaw > 0:
        adj_yaw = math.pi-yaw  # adjusted yaw for proportional control
    else:
        adj_yaw = -math.pi-yaw
    P_const = 5  # constant for proportional control
    basevel = 3  # target velocity
    p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=0, controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=basevel - P_const * adj_yaw, force=driving_torque)
    p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=1, controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=basevel + P_const * adj_yaw, force=driving_torque)
    if climb_attempts == 1:
        if climb_successes == 1:
            # the below variable can be changed; currently, a parameter sweep is conducted for step height
            step_height += 0.001
            for stair in stairs:
                p.removeBody(stair)
            stairs = []
            for j in range(num_steps):
                x = j * step_depth
                z = (j + 0.5) * step_height
                collision = p.createCollisionShape(
                    p.GEOM_BOX, halfExtents=[step_depth/2, step_width/2, step_height/2])
                visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[
                                             step_depth/2, step_width/2, step_height/2], rgbaColor=[0.5, 0.5, 0.5, 1])
                stairId = p.createMultiBody(
                    baseMass=0, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, basePosition=[x, 0, z])
                stairs.append(stairId)
            for stair in stairs:
                p.changeDynamics(stair, -1, lateralFriction=stair_friction)
            for i in range(num_joints):
                p.changeDynamics(robotId, i, lateralFriction=stair_friction)
            p.resetBasePositionAndOrientation(
                robotId, startPos, startOrientation)  # the second reset is mainly used to prevent discrepancies when testing friction
            for i in range(num_joints):
                p.resetJointState(robotId, i, targetValue=0.0,
                                  targetVelocity=0.0)
            p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=0,
                                    controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=driving_torque)
            p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=1,
                                    controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=driving_torque)
            for i in range(120):
                p.stepSimulation()
                time.sleep(1/(240*speedup_factor))
            climb_start = sim_steps
        else:
            # the parameter sweep stops when failure occurs
            print("Limiting height: " + str(step_height - 0.001))
            break
        climb_attempts = 0
        climb_successes = 0
    if climb_end - climb_start > 5000:
        # if 5000 simulation steps have passed without climb, then attempt failed
        print("Climb attempt failed.")
        print(step_height)
        climb_attempts += 1
        p.resetBasePositionAndOrientation(robotId, startPos, startOrientation)
        for i in range(num_joints):
            p.resetJointState(robotId, i, targetValue=0.0, targetVelocity=0.0)
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=0,
                                controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=driving_torque)
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=1,
                                controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=driving_torque)
        for i in range(120):
            p.stepSimulation()
            time.sleep(1/(240*speedup_factor))
        climb_start = sim_steps
    if left_state[0][2] > num_steps*step_height+0.1 or right_state[0][2] > num_steps*step_height+0.1:
        # condition for climb successs
        print("Climb attempt success.")
        print(step_height)
        climb_attempts += 1
        climb_successes += 1
        p.resetBasePositionAndOrientation(robotId, startPos, startOrientation)
        for i in range(num_joints):
            p.resetJointState(robotId, i, targetValue=0.0, targetVelocity=0.0)
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=0,
                                controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=driving_torque)
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=1,
                                controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=driving_torque)
        for i in range(120):
            p.stepSimulation()
            time.sleep(1/(240*speedup_factor))
        climb_start = sim_steps
    if p.isConnected():
        p.stepSimulation()
        sim_steps += 1
    else:
        break
    time.sleep(1/(240*speedup_factor))
p.disconnect()