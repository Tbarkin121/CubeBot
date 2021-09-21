import pybullet as p
import pybullet_data
from time import sleep
import subprocess

# Can alternatively pass in p.DIRECT 
client = p.connect(p.GUI)
p.setGravity(0, 0, -10, physicsClientId=client) 

wheel1 = p.addUserDebugParameter('Wheel 1', -100, 100, 0)
wheel2 = p.addUserDebugParameter('Wheel 2', -100, 100, 0)
wheel3 = p.addUserDebugParameter('Wheel 3', -100, 100, 0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
# laikago = p.loadURDF("laikago/laikago.urdf", basePosition=[0,0,1])
# plane = p.loadURDF("simple_driving/resources/simpleplane.urdf") 
robot = p.loadURDF("box_pendulum/resources/cubebot_3D.urdf", basePosition=[0,0,1.5])


number_of_joints = p.getNumJoints(robot)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(robot,joint_number)
    print(info[0], ": ", info[1] )
    print('Joint Damping = {}'.format(info[6]))
    print('Joint Friction = {}'.format(info[7]))


corner_indices = [0, 1, 2, 3, 4, 5, 6, 7]
wheel_indices = [8, 9, 10, 11, 12, 13]
corner_pos_old = [0, 0, 0, 0, 0, 0, 0, 0]

p.changeDynamics(robot, -1, linearDamping=0, angularDamping=0, maxJointVelocity=1000, mass=0.1)

for wheel in wheel_indices:
    # p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, force=0, maxVelocity=1000)
    p.setJointMotorControl2(robot, wheel, p.VELOCITY_CONTROL, force=0)
    p.enableJointForceTorqueSensor(robot, wheel, enableSensor=1)
    # p.changeDynamics(robot, wheel_indices[0], mass=1.0, lateralFriction=0.0, spinningFriction=0.0, rollingFriction=0.0, jointDamping=0.0, angularDamping=0)
    DynamicsInfo = p.getDynamicsInfo(robot, wheel)
    # print(DynamicsInfo[2])
    p.changeDynamics(robot, wheel, angularDamping=0, maxJointVelocity=1000, mass=.1)
    p.changeDynamics(robot, wheel, localInertiaDiagonal=[DynamicsInfo[2][0]*1, DynamicsInfo[2][1]*1, DynamicsInfo[2][2]*1])


DynamicsInfo = p.getDynamicsInfo(robot, wheel_indices[0])
print("Dynamics Info : ")
print(DynamicsInfo)
print('Mass : {}'.format(DynamicsInfo[0]))
print('Lateral Friction : {}'.format(DynamicsInfo[1]))

for corner in corner_indices:
    p.changeVisualShape(robot, corner, rgbaColor=[1.0, 1.0, 1.0, 1.0])

loop_idx = 0
for _ in range(10000): 
    user_wheel1 = p.readUserDebugParameter(wheel1)
    user_wheel2 = p.readUserDebugParameter(wheel2)
    user_wheel3 = p.readUserDebugParameter(wheel3)

    # if(loop_idx % 20 == 0):
    data = p.getLinkStates(robot, corner_indices)
    for corner in corner_indices:
        if (data[corner][0][2] > 0.5 and corner_pos_old[corner] < 0.5):
            p.changeVisualShape(robot, corner, rgbaColor=[0.0, 1.0, 0.0, 1.0])
        if (data[corner][0][2] < 0.5 and corner_pos_old[corner] > 0.5):
            p.changeVisualShape(robot, corner, rgbaColor=[1.0, 0.0, 0.0, 1.0])
        corner_pos_old[corner] = data[corner][0][2]
    loop_idx += 1 


    p.setJointMotorControl2(robot, wheel_indices[0],
                            p.VELOCITY_CONTROL,
                            targetVelocity=user_wheel1,
                            force=50)

    p.setJointMotorControl2(robot, wheel_indices[1],
                            p.VELOCITY_CONTROL,
                            targetVelocity=user_wheel1,
                            force=50)
                            

    p.setJointMotorControl2(robot, wheel_indices[2],
                            p.VELOCITY_CONTROL,
                            targetVelocity=user_wheel2,
                            force=50)
    
    p.setJointMotorControl2(robot, wheel_indices[3],
                            p.VELOCITY_CONTROL,
                            targetVelocity=user_wheel2,
                            force=50)

    p.setJointMotorControl2(robot, wheel_indices[4],
                            p.VELOCITY_CONTROL,
                            targetVelocity=user_wheel3,
                            force=50)

    p.setJointMotorControl2(robot, wheel_indices[5],
                            p.VELOCITY_CONTROL,
                            targetVelocity=user_wheel3,
                            force=50)


    # pos, ori = p.getBasePositionAndOrientation(carId)
    # p.applyExternalForce(carId, 0, [150, 0, 0], pos, p.WORLD_FRAME)

    for wheel_joint in wheel_indices:
        joint_states = p.getJointState(robot, wheel_joint)
        # print('Joint Position = {}'.format(joint_states[0]))
        # print('Joint Velocity = {}'.format(joint_states[1]))
        # print('Joint Reaction Forces = \n{}\n{}\n{}\n{}\n{}\n{}'.format(joint_states[2][0],joint_states[2][1],joint_states[2][2],joint_states[2][3],joint_states[2][4],joint_states[2][5]))
        # print('Joint Motor Torque = {}'.format(joint_states[3]))

    p.stepSimulation()
    sleep(1/240)

p.disconnect()
