import gym
import numpy as np
import math
import pybullet as p
from box_pendulum.resources.cubebot import CubeBot
import matplotlib.pyplot as plt


class CubeStandEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.action_space = gym.spaces.box.Box(
            low=np.array([-1], dtype=np.float32), #Throttle
            high=np.array([1], dtype=np.float32))
        self.observation_space = gym.spaces.box.Box(
            low=np.array([-4, -1, -1, -4], dtype=np.float32), # HingeAngle, HingeVel_scaled, FlywheelVel_scaled, Goal 
            high=np.array([4, 1, 1, -4], dtype=np.float32)) 
        self.np_random, _ = gym.utils.seeding.np_random()
        
        # self.client = p.connect(p.GUI)
        self.client = p.connect(p.DIRECT)
        
        self.user_angle = None
        self.user_throttle = None

        # Reduce length of episodes for RL algorithms
        p.setTimeStep(1/240, self.client)

        self.robot = None
        self.goal = None
        self.done = False
        self.rendered_img = None
        self.render_rot_matrix = None
        self.reset()

        self.max_step_count = 2500
        self.current_step_count = 0
        
    def step(self, action):
        # Feed action to the robot and get observation of robot's state
        # action = self.get_user_action()
        self.robot.apply_action(action)
        p.stepSimulation()
        robot_ob = self.robot.get_observation()

        # Compute reward as L2 change in distance to goal
        error = (robot_ob[0] - self.goal[0])
        error2 = (robot_ob[0] - self.goal[0]) ** 2
        reward = 1 - error2
        # print(reward)
        ob = np.array(robot_ob + self.goal, dtype=np.float32)

        if(self.current_step_count > self.max_step_count):
            reward = 0
            self.done = True
        self.current_step_count += 1
        
        return ob, reward, self.done, dict()

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -10)
        # Reload the Robot
        self.robot = CubeBot(self.client)

        # Set the goal to a random target
        self.reset_goal()
        self.done = False

        # # Visual element of the goal
        # self.goalObj = Goal(self.client, self.goal)

        # Get observation to return
        robot_ob = self.robot.get_observation()

        self.current_step_count = 0

        return np.array(robot_ob + self.goal, dtype=np.float32)

    def reset_goal(self):
        # The goal is a body angle, starting with 45 deg (upright)
        goal_variance = np.random.uniform(low=-0.3, high=0.3)
        self.goal = (0.785398+goal_variance,) #45 deg in rad

        # print("p.getNumBodies : {}".format(p.getNumBodies()))
        # for i in range(p.getNumBodies()):
        #     print("p.getBodyInfo(i) : {}".format(p.getBodyInfo(i)))


    def render(self, mode='human'):
        if self.rendered_img is None:
            self.rendered_img = plt.imshow(np.zeros((100, 100, 4)))

        # Base information
        robot_id, client_id = self.robot.get_ids()
        proj_matrix = p.computeProjectionMatrixFOV(fov=80, aspect=1,
                                                   nearVal=0.01, farVal=100)
        pos, ori = [list(l) for l in
                    p.getBasePositionAndOrientation(robot_id, client_id)]
        pos[2] = 0.2

        # Rotate camera direction
        rot_mat = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
        camera_vec = np.matmul(rot_mat, [1, 0, 0])
        camera_offset = np.matmul(rot_mat, [2, 0, 1])
        up_vec = np.matmul(rot_mat, np.array([0, 0, 1]))
        view_matrix = p.computeViewMatrix(pos+camera_offset, pos, up_vec)

        # Display image
        frame = p.getCameraImage(100, 100, view_matrix, proj_matrix)[2]
        frame = np.reshape(frame, (100, 100, 4))
        self.rendered_img.set_data(frame)
        plt.draw()
        plt.pause(.00001)

    def get_user_action(self):
        self.user_angle = p.readUserDebugParameter(self.angle)
        self.user_throttle = p.readUserDebugParameter(self.throttle)
        return ([self.user_throttle , self.user_angle])

    def close(self):
        p.disconnect(self.client)
