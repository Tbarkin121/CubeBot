import pybullet as p
import numpy as np
import os
import math


class CubeBot:
    def __init__(self, client):
        self.client = client
        f_name = os.path.join(os.path.dirname(__file__), 'cubebot.urdf')
        self.robot = p.loadURDF(fileName=f_name,
                              basePosition=[0, 0, 0.1],
                              baseOrientation=[0, 0, 0, 1],
                              physicsClientId=client)

        # Joint indices as found by p.getJointInfo()
        self.hinge_joint = [1]
        self.wheel_joint = [2]
        self.max_wheel_vel = 50
        self.max_hinge_vel = 10

        self.start_angle = np.random.uniform(low=0.0, high=1.5708)
        self.start_velocity = 0.0
        p.resetJointState(self.robot, 
                          self.hinge_joint[0], 
                          targetValue=self.start_angle, 
                          targetVelocity=self.start_velocity, 
                          physicsClientId=self.client)

        for i in range(p.getNumJoints(self.robot)):
            p.setJointMotorControl2(self.robot, i, p.VELOCITY_CONTROL, force=0)
        p.enableJointForceTorqueSensor(self.robot, self.wheel_joint[0], enableSensor=1)
        p.changeDynamics(self.robot, self.wheel_joint[0], angularDamping=0, maxJointVelocity=self.max_wheel_vel)

        # Joint speed
        self.joint_speed = 0
        # Drag constants
        self.c_rolling = 0.0
        self.c_drag = 0.0
        # Throttle constant increases "speed" of the car
        self.max_torque = 20
        self.c_throttle = 250

    def get_ids(self):
        return self.robot, self.client

    def apply_action(self, action):
        # Expects action to be two dimensional
        throttle = action
        
        # Clip throttle and steering angle to reasonable values
        throttle = min(max(throttle, -1), 1)
    
        
        # Calculate drag / mechanical resistance ourselves
        # Using velocity control, as torque control requires precise models
        friction = -self.joint_speed * (np.sign(self.joint_speed)*self.joint_speed * self.c_drag +
                                        self.c_rolling)
        acceleration = self.c_throttle * throttle + friction
        # Each time step is 1/240 of a second
        self.joint_speed = self.joint_speed + 1/240 * acceleration
        if(abs(self.joint_speed) > self.max_wheel_vel):
            self.joint_speed = np.sign(self.joint_speed)*self.max_wheel_vel
        # print(self.joint_speed)
        # Set the velocity of the wheel joints directly
        p.setJointMotorControl2(self.robot, self.wheel_joint[0],
                                p.VELOCITY_CONTROL,
                                targetVelocity=self.joint_speed,
                                force=self.max_torque,
                                physicsClientId=self.client)

    def get_observation(self):
        # Get the joint state for the flywheel. There is only 1 right now
        wheel_joint_states = p.getJointState(self.robot, self.wheel_joint[0])
        wheel_velocity = wheel_joint_states[1]/self.max_wheel_vel
        hinge_joint_state = p.getJointState(self.robot, self.hinge_joint[0])
        hinge_position = hinge_joint_state[0]
        hinge_velocity = hinge_joint_state[1]/self.max_hinge_vel
        observation = (hinge_position, hinge_velocity, wheel_velocity)
        return observation









