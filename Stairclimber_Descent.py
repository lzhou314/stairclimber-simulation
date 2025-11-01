#Created by Liran Zhou
import pybullet as p
import time
import math
import pybullet_data
physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
p.setPhysicsEngineParameter(numSolverIterations=200)
p.setRealTimeSimulation(0)
p.setTimeStep(1/240)
planeId = p.loadURDF("plane.urdf")
step_width = 1 # all length units are in meters
step_height = 0.15
step_depth = 0.23
num_steps = 15
stairs = []
for i in range(num_steps):
    # position of step center
    x = i * step_depth
    z = (i+0.5) * step_height
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[step_depth/2, step_width/2, step_height/2])
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[step_depth/2, step_width/2, step_height/2], rgbaColor=[0.5, 0.5, 0.5, 1])
    stairId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, basePosition=[x, 0, z])
    stairs.append(stairId)
startPos = [num_steps*step_depth-0.45,0, num_steps*step_height+0.2]
theta = math.atan(step_height/step_depth)
startOrientation = p.getQuaternionFromEuler([0,theta,3.14159])
boxId=p.loadURDF("/Users/liranz/Downloads/testing5.urdf", startPos, startOrientation) # change the driving_torquele based on current design being tested 
p.resetDebugVisualizerCamera(
    cameraDistance=4,            
    cameraYaw=0,              
    cameraPitch=-15,             
    cameraTargetPosition=[1.6, 0, 1.2]
)
p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
num_joints = p.getNumJoints(boxId)
left_state = p.getLinkState(boxId, 0)
right_state = p.getLinkState(boxId, 1)
stair_friction = 1.0 # friction coefdriving_torquecient between wheel and stairs
base_friction = 0.3 # friction coefdriving_torquecient between base and other objects
p.changeDynamics(planeId, -1, lateralFriction=1.0)
p.changeDynamics(boxId, -1, lateralFriction=base_friction)  # base
for stair in stairs:
    p.changeDynamics(stair, -1, lateralFriction=stair_friction)
for i in range(num_joints):
    p.changeDynamics(boxId, i, lateralFriction=stair_friction)
p.resetBaseVelocity(boxId, [0,0,0], [0,0,0])
for j in range(num_joints):
    p.resetJointState(boxId, j, targetValue=0, targetVelocity=0)
for i in range(240):
    p.stepSimulation()
    time.sleep(1/240)
p.setGravity(0, 0, -10)
driving_torque = 6.3
p.setJointMotorControl2(bodyUniqueId=boxId, jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity=-3, force=driving_torque)
p.setJointMotorControl2(bodyUniqueId=boxId, jointIndex=1, controlMode=p.VELOCITY_CONTROL, targetVelocity=-3, force=driving_torque)
down_start = 0
# used to keep track of how many simulation steps passed since beginning of trial
down_attempts = 0
down_successes = 0
speedup_factor = 300 #factor to which simulation is sped up by
sim_steps = 0
while True:
    left_state = p.getLinkState(boxId, 0)
    right_state = p.getLinkState(boxId, 1)
    pos, orn = p.getBasePositionAndOrientation(boxId)
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)
    if yaw>0:
        adj_yaw = math.pi-yaw # adjusted yaw for proportional control
    else:
        adj_yaw = -math.pi-yaw
    P_const = 5 # constant for proportional control
    base_vel = -3 # target velocity
    p.setJointMotorControl2(bodyUniqueId=boxId, jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity=base_vel - P_const * adj_yaw, force=driving_torque)
    p.setJointMotorControl2(bodyUniqueId=boxId, jointIndex=1, controlMode=p.VELOCITY_CONTROL, targetVelocity=base_vel + P_const * adj_yaw, force=driving_torque)
    if down_attempts == 1:
        if down_successes == 1:
            # the below variable can be changed; currently, a parameter sweep is conducted for friction between wheel and stair
            stair_friction -= 0.005
            for stair in stairs:
                p.changeDynamics(stair, -1, lateralFriction=stair_friction)
            for i in range(num_joints):
                p.changeDynamics(boxId, i, lateralFriction=stair_friction)
            startPos = [num_steps*step_depth-0.45,0, num_steps*step_height+0.2]
            theta = math.atan(step_height/step_depth)
            startOrientation = p.getQuaternionFromEuler([0,theta,3.14159])
            p.resetBasePositionAndOrientation(boxId, startPos, startOrientation) # the second reset is mainly used to prevent discrepancies when testing friction
            for i in range(num_joints):
                p.resetJointState(boxId, i, targetValue=0.0, targetVelocity=0.0)
            p.setJointMotorControl2(bodyUniqueId=boxId, jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=driving_torque)
            p.setJointMotorControl2(bodyUniqueId=boxId, jointIndex=1, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=driving_torque)
            for i in range(240):
                p.stepSimulation()
                time.sleep(1/(240*speedup_factor))
            down_start = sim_steps
        else:
            print("Limiting friction: "+ str(stair_friction + 0.005))
            break
        down_attempts = 0
        down_successes = 0
    if left_state[0][2] < 0.3 or right_state[0][2] < 0.3:
        down_end = sim_steps
        down_attempts+=1
        if down_end - down_start > 1500:
            # if it descends in less than 1500 simulation steps, then descent fails
            print("Down attempt success.")
            print(stair_friction)
            down_successes+=1
        else:
            print("Down attempt failed.")
            print(stair_friction)
        p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
        for i in range(num_joints):
            p.resetJointState(boxId, i, targetValue=0.0, targetVelocity=0.0)
        p.setJointMotorControl2(bodyUniqueId=boxId, jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=driving_torque)
        p.setJointMotorControl2(bodyUniqueId=boxId, jointIndex=1, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=driving_torque)
        for i in range(240):
            p.stepSimulation()
            time.sleep(1/(240*speedup_factor))
        down_start = sim_steps
    if p.isConnected():
        p.stepSimulation()
        sim_steps += 1
    else:
        break
    time.sleep(1/(240*speedup_factor))
p.disconnect()