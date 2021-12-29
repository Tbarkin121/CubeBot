import pybullet as p
import pybullet_data
from time import sleep
import subprocess

# Can alternatively pass in p.DIRECT 
client = p.connect(p.GUI)
p.setTimeStep(1/10000, client)
p.setGravity(0, 0, -10, physicsClientId=client) 

angle = p.addUserDebugParameter('Steering', -15, 15, 0)
wheel_torque = p.addUserDebugParameter('Torque', -1000, 1000, 0)



p.setAdditionalSearchPath(pybullet_data.getDataPath())
# planeId = p.loadURDF("plane.urdf")
# laikago = p.loadURDF("laikago/laikago.urdf", basePosition=[0,0,1])
# plane = p.loadURDF("simple_driving/resources/simpleplane.urdf") 
robot = p.loadURDF("box_pendulum/resources/gyro.urdf", basePosition=[0.0, 0.0, 1.0])


number_of_joints = p.getNumJoints(robot)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(robot,joint_number)
    print(info[0], ": ", info[1] )
    print('Joint Damping = {}'.format(info[6]))
    print('Joint Friction = {}'.format(info[7]))



wheel_indices = [2]


for i in range(p.getNumJoints(robot)):
    # p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, force=0, maxVelocity=1000)
	p.setJointMotorControl2(robot, i, p.VELOCITY_CONTROL, force=0)

p.enableJointForceTorqueSensor(robot, wheel_indices[0], enableSensor=1)
# p.changeDynamics(robot, wheel_indices[0], mass=1.0, lateralFriction=0.0, spinningFriction=0.0, rollingFriction=0.0, jointDamping=0.0, angularDamping=0)
p.changeDynamics(robot, wheel_indices[0], angularDamping=0, maxJointVelocity=1000)

DynamicsInfo = p.getDynamicsInfo(robot, wheel_indices[0])
print("Dynamics Info : ")
print(DynamicsInfo)
print('Mass : {}'.format(DynamicsInfo[0]))
print('Lateral Friction : {}'.format(DynamicsInfo[1]))


for _ in range(100000): 
    user_torque = p.readUserDebugParameter(wheel_torque)
    for joint_index in wheel_indices:
        # p.setJointMotorControl2(robot, joint_index,
        #                         p.TORQUE_CONTROL,
        #                         force=user_torque)

        p.setJointMotorControl2(robot, joint_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=user_torque,
                                force=20)


    # pos, ori = p.getBasePositionAndOrientation(carId)
    # p.applyExternalForce(carId, 0, [150, 0, 0], pos, p.WORLD_FRAME)

    for wheel_joint in wheel_indices:
        joint_states = p.getJointState(robot, wheel_joint)
        print('Joint Position = {}'.format(joint_states[0]))
        print('Joint Velocity = {}'.format(joint_states[1]))
        print('Joint Reaction Forces = \n{}\n{}\n{}\n{}\n{}\n{}'.format(joint_states[2][0],joint_states[2][1],joint_states[2][2],joint_states[2][3],joint_states[2][4],joint_states[2][5]))
        print('Joint Motor Torque = {}'.format(joint_states[3]))

    p.stepSimulation()
    sleep(1/10000)

p.disconnect()
