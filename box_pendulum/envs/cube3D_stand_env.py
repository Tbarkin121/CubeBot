import gym
import numpy as np
import math
import pybullet as p
from box_pendulum.resources.cubebot_3D import CubeBot3D
from box_pendulum.resources.plane import Plane
import matplotlib.pyplot as plt


class Cube3DStandEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        # Reaction Wheel Throttles : Vec3 (There are three unique wheel inputs)
        self.action_space = gym.spaces.box.Box(low=-1,
                                               high=1, 
                                               shape=(3,),
                                               dtype=np.float32) 

        # Base Orientation : Vec4
        # Base Velocity : Vec3
        # Flywheel Speeds : Vec3
        # Goal : None (Reward will be based on Base Orientation to start)
        self.observation_space = gym.spaces.box.Box(low=-1,
                                                    high=1, 
                                                    shape=(10,),
                                                    dtype=np.float32) 

        self.np_random, _ = gym.utils.seeding.np_random()
        
        self.client = p.connect(p.GUI)
        # self.client = p.connect(p.DIRECT)
        
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

        # The reward is based on the corner Z-height
        # This structure should incentivise getting the corner as high as possible
        # I expect the bot to learn how to ballance on an edge, and maybe later a corner. 
        
        # Scaling with the height of the robot, (Because I changed from a 1m edge to a 0.3m edge recently)
        reward = self.robot.get_corner_height()/self.robot.bodyShape[0][3][0]
        # print(reward)
        ob = np.array(robot_ob, dtype=np.float32)

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
        p.setGravity(0, 0, -9.81)
        # Reload the Plane
        Plane(self.client)
        # Reload the Robot
        self.robot = CubeBot3D(self.client)

        # Set the goal to a random target
        self.reset_goal()
        self.done = False

        # # Visual element of the goal
        # self.goalObj = Goal(self.client, self.goal)

        # Get observation to return
        robot_ob = self.robot.get_observation()

        self.current_step_count = 0
        # print(robot_ob)
        return np.array(robot_ob, dtype=np.float32)

    def reset_goal(self):
        # The goal is a body angle, starting with 45 deg (upright)
        self.goal = (0.785398,) #45 deg in rad

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
        # pos[2] = 0.2

        # Rotate camera direction
        rot_mat = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
        camera_vec = np.matmul(rot_mat, [1, 0, 0])
        # camera_offset = np.matmul(rot_mat, [2, 0, 1])
        camera_offset = np.array([0.1, 0, 1])
        # up_vec = np.matmul(rot_mat, np.array([0, 0, 1]))
        up_vec = [0, 0, 1]
        # view_matrix = p.computeViewMatrix(pos+camera_offset, pos, up_vec)
        print(pos+camera_offset)
        view_matrix = p.computeViewMatrix(pos+camera_offset, pos, up_vec)

        # Display image
        frame = p.getCameraImage(200, 200, view_matrix, proj_matrix)[2]
        frame = np.reshape(frame, (200, 200, 4))
        self.rendered_img.set_data(frame)
        plt.draw()
        plt.pause(.00001)

    def get_user_action(self):
        self.user_angle = p.readUserDebugParameter(self.angle)
        self.user_throttle = p.readUserDebugParameter(self.throttle)
        return ([self.user_throttle , self.user_angle])

    def close(self):
        p.disconnect(self.client)
