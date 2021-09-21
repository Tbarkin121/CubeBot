import pybullet as p
import numpy as np
import os
import math

class CubeBot3D:
    def __init__(self, client):
        self.client = client
        f_name = os.path.join(os.path.dirname(__file__), 'cubebot_3D.urdf')
        self.robot = p.loadURDF(fileName=f_name,
                              basePosition=[0, 0, 0.8],
                              baseOrientation=[0, 0, 0, 1],
                              physicsClientId=client)
        
        # Joint indices as found by p.getJointInfo()
        self.corner_indices = [0, 1, 2, 3, 4, 5, 6, 7]
        self.wheel_indices = [8, 9, 10, 11, 12, 13]

        # corner pos old is used for coloring the cube corners.
        # reduces the calls required to set color
        self.corner_pos_old = [0, 0, 0, 0, 0, 0, 0, 0]

        p.changeVisualShape(self.robot, self.corner_indices[0], rgbaColor=[0.0, 1.0, 0.0, 1.0])
        p.changeDynamics(self.robot, -1, linearDamping=0, angularDamping=0, maxJointVelocity=1000)
        for wheel in self.wheel_indices:
            # p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, force=0, maxVelocity=1000)
            p.setJointMotorControl2(self.robot, wheel, p.VELOCITY_CONTROL, force=0)
            p.enableJointForceTorqueSensor(self.robot, wheel, enableSensor=1)
            # p.changeDynamics(robot, wheel, mass=1.0, lateralFriction=0.0, spinningFriction=0.0, rollingFriction=0.0, jointDamping=0.0, angularDamping=0)
            p.changeDynamics(self.robot, wheel, mass=0.1, linearDamping=0, angularDamping=0, maxJointVelocity=1000)

            DynamicsInfo = p.getDynamicsInfo(self.robot, wheel)
            p.changeDynamics(self.robot, wheel, localInertiaDiagonal=[DynamicsInfo[2][0]*2, DynamicsInfo[2][1]*2, DynamicsInfo[2][2]*2])
            
        # wheel speed
        self.wheel_speed = [0.0, 0.0, 0.0]
        # Drag constants
        self.c_rolling = 0.0
        self.c_drag = 0.0
        # Throttle constant increases "speed" of the car
        self.max_torque = 20
        self.c_throttle = 300
        self.max_wheel_vel = 100

    def get_ids(self):
        return self.robot, self.client

    def apply_action(self, action):
        # Expects action to be three dimensional
        # Action applied 
        # print('action = {}'.format(action))
        self.throttle_control(action, wheel_num=0)
        self.throttle_control(action, wheel_num=1)
        self.throttle_control(action, wheel_num=2)
        

    def throttle_control(self, action, wheel_num):
        # Clip throttle to reasonable values

        throttle_clip = min(max(action[wheel_num], -1), 1)

        # # Calculate drag / mechanical resistance ourselves
        # # Using velocity control, as torque control requires precise models
        # wheel_speed=self.wheel_speed[wheel_num]
        # friction = -wheel_speed * (np.sign(wheel_speed)*wheel_speed * self.c_drag +
        #                                 self.c_rolling)
        # acceleration = self.c_throttle * throttle_clip + friction
        # # Each time step is 1/240 of a second
        # self.wheel_speed[wheel_num] += 1/240 * acceleration

        # if(abs(self.wheel_speed[wheel_num]) > self.max_wheel_vel):
        #     self.wheel_speed[wheel_num] = np.sign(self.wheel_speed[wheel_num])*self.max_wheel_vel
        # # print(self.wheel_speed[wheel_num])
        # # Set the velocity of the wheel joints directly
        # # Modify the index, there are two wheels per axis right now
        self.wheel_speed[wheel_num]=throttle_clip*self.max_wheel_vel
        p.setJointMotorControl2(self.robot, self.wheel_indices[wheel_num*2],
                                p.VELOCITY_CONTROL,
                                targetVelocity=self.wheel_speed[wheel_num],
                                force=self.max_torque,
                                physicsClientId=self.client)

        p.setJointMotorControl2(self.robot, self.wheel_indices[wheel_num*2+1],
                                p.VELOCITY_CONTROL,
                                targetVelocity=self.wheel_speed[wheel_num],
                                force=self.max_torque,
                                physicsClientId=self.client)

    def get_observation(self):
        # Get Base State Information
        BasePosition = p.getBasePositionAndOrientation(self.robot,
                                                       physicsClientId=self.client)
        BaseVelocity = p.getBaseVelocity(self.robot,
                                         physicsClientId=self.client)

        # Get the joint state for the flywheel. There are 6, but only 3 are unique
        WheelState1 = p.getJointState(self.robot, 
                                     self.wheel_indices[0],
                                     physicsClientId=self.client)
        WheelState2 = p.getJointState(self.robot, 
                                     self.wheel_indices[2],
                                     physicsClientId=self.client)
        WheelState3 = p.getJointState(self.robot, 
                                     self.wheel_indices[4],
                                     physicsClientId=self.client)

        WheelVel1 = WheelState1[1]/self.max_wheel_vel
        WheelVel2 = WheelState2[1]/self.max_wheel_vel
        WheelVel3 = WheelState3[1]/self.max_wheel_vel

        # Only concerned with orientation for now
        observation = (BasePosition[1][0], BasePosition[1][1], BasePosition[1][2], BasePosition[1][3], 
                       BaseVelocity[1][0], BaseVelocity[1][1], BaseVelocity[1][2],
                       WheelVel1, WheelVel2, WheelVel3)
        return observation

    def get_corner_height(self):
        LinkState = p.getLinkState(self.robot, self.corner_indices[0])
        return LinkState[0][2]







