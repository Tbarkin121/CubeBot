import pybullet as p
import pybullet_data
from time import sleep
import subprocess

# Can alternatively pass in p.DIRECT 
client = p.connect(p.GUI)
p.setTimeStep(1/10000, client)
p.setGravity(0, 0, 0, physicsClientId=client) 

angle = p.addUserDebugParameter('Steering', -15, 15, 0)
wheel_torque = p.addUserDebugParameter('Torque', -10, 10, 0)



p.setAdditionalSearchPath(pybullet_data.getDataPath())
# planeId = p.loadURDF("plane.urdf")
# laikago = p.loadURDF("laikago/laikago.urdf", basePosition=[0,0,1])
# plane = p.loadURDF("simple_driving/resources/simpleplane.urdf") 
robot = p.loadURDF("box_pendulum/resources/t_handle.urdf", basePosition=[0.0, 0.0, 1.0])
p.changeDynamics(robot, 0, angularDamping=0, maxJointVelocity=1000)

number_of_joints = p.getNumJoints(robot)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(robot,joint_number)
    print(info[0], ": ", info[1] )
    print('Joint Damping = {}'.format(info[6]))
    print('Joint Friction = {}'.format(info[7]))


step_counter = 0
for _ in range(1000000): 
    user_torque = p.readUserDebugParameter(wheel_torque)
    if (step_counter<1000):
        p.applyExternalTorque(robot, -1, [0, 0, 10], flags=p.WORLD_FRAME)
    step_counter += 1
    p.stepSimulation()
    sleep(1/10000)

p.disconnect()
