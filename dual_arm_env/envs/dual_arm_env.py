import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print("current_dir=" + currentdir)
os.sys.path.insert(0, currentdir)

import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import pybullet as p

import random
import pybullet_data
from pkg_resources import parse_version
from dual_arm_env.resources.dualarm import DualArm

largeValObservation = 100

RENDER_HEIGHT = 720
RENDER_WIDTH = 960


class DualArmEnv(gym.Env):
  metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

  def __init__(self,
               urdfRoot=currentdir+"/../urdf",
               actionRepeat=1,
               isEnableSelfCollision=True,
               renders=False,
               isDiscrete=False,
               maxSteps=1000):
    self._isDiscrete = isDiscrete
    self._timeStep = 1. / 240.
    self._urdfRoot = urdfRoot
    self._actionRepeat = actionRepeat
    self._isEnableSelfCollision = isEnableSelfCollision
    self._observation = []
    self._envStepCounter = 0
    self._renders = renders
    self._maxSteps = maxSteps
    self.terminated = 0
    self._cam_dist = 1.3
    self._cam_yaw = 180
    self._cam_pitch = -40

    self._p = p  

    if self._renders:
      cid = p.connect(p.SHARED_MEMORY)
      if (cid < 0):
        cid = p.connect(p.GUI)
      p.resetDebugVisualizerCamera(3, 89.999, -41, [0,0,0])
    else:
      p.connect(p.DIRECT)
    self.seed()
    self.reset()


    observationDim = 17
    largeValObservation = 1
    observation_high = np.array([largeValObservation] * observationDim)

    action_dim = 12
    self._action_bound = np.pi
    action_high = np.array([self._action_bound] * action_dim)
    self.action_space = spaces.Box(-action_high, action_high)
    self.observation_space = spaces.Box(-observation_high, observation_high)
    self.viewer = None

  def reset(self):
    self.terminated = 0
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=150)
    p.setTimeStep(self._timeStep)
    p.loadURDF(os.path.join(self._urdfRoot, "simpleplane.urdf"), [0, 0, -0.1])
    p.setGravity(0, 0, -10)
    print(self._urdfRoot)
    xpos = 0.3 + 0.3 * random.random()-0.15
    ypos = 0.0 + 0.5 * random.random()-0.25
    zpos = 0.5 + 0.3 * random.random()
    orn = p.getQuaternionFromEuler([0, 0, 0])
    self.blockUid = p.loadURDF(os.path.join(self._urdfRoot, "box.urdf"), xpos, ypos, zpos,
                               orn[0], orn[1], orn[2], orn[3])
    self._dualarm = DualArm(urdfRootPath=self._urdfRoot, timeStep=self._timeStep)
    self._envStepCounter = 0
    p.stepSimulation()
    self._observation = self.getExtendedObservation()
    return np.array(self._observation)


  def __del__(self):
    p.disconnect()


  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def getExtendedObservation(self):
    self._observation = self._dualarm.getObservation()
    blockPos, blockOrn = p.getBasePositionAndOrientation(self.blockUid)
    left_state = p.getLinkState(self._dualarm.dualarmUid, self._dualarm.left_eef_index)
    left_eef_pos = left_state[0]
    left_eef_orn = left_state[1]
    right_state = p.getLinkState(self._dualarm.dualarmUid, self._dualarm.right_eef_index)
    right_eef_pos = right_state[0]
    right_eef_orn = right_state[1]
    return self._observation.extend(list(blockPos))

  def step(self, action):
      realAction = action;
      return self.step2(realAction)

  def getCameraImage(self, cam_pos,cam_orn):
    near = 0.01
    far = 1000
    fov = 60
    aspect = RENDER_WIDTH/RENDER_HEIGHT
    angle = 0.0;
    q = p.getQuaternionFromEuler(cam_orn)
    cam_orn = np.reshape(p.getMatrixFromQuaternion(q ),(3,3));
    cam_orn = [cam_orn[0]-1.5707,cam_orn[1],cam_orn[2]+3.141592]
    view_pos = np.matmul(cam_orn,np.array([-0.001,0,0.0]).T)
    view_pos = np.array(view_pos+cam_pos)
    view_matrix = p.computeViewMatrix([cam_pos[0],cam_pos[1],cam_pos[2]], view_pos, [0,0,1])
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    (_, _, px, _, _)  = p.getCameraImage(RENDER_WIDTH,
            RENDER_HEIGHT,
            view_matrix,
            projection_matrix,
            shadow=False)
    return px
  def step2(self, action):
    for i in range(self._actionRepeat):
      self._dualarm.applyAction(action)
      p.stepSimulation()
      if self._termination():
        break
      self._envStepCounter += 1
    if self._renders:
      time.sleep(self._timeStep)
    self._observation = self.getExtendedObservation()
    done = self._termination()
    reward = self._reward();
    return np.array(self._observation), reward, done, {}

  def render(self, mode="rgb_array", close=False):
    if mode != "rgb_array":
      return np.array([])
    camera_T = p.getLinkState(self._dualarm.dualarmUid,self._dualarm.camera_index)

    rgb_array = self.getCameraImage(camera_T[0],p.getEulerFromQuaternion(camera_T[1]))
    rgb_array = np.array(rgb_array, dtype=np.uint8)
    rgb_array = np.reshape(rgb_array, (RENDER_HEIGHT, RENDER_WIDTH, 4))
    rgb_array = rgb_array[:, :, :3]
    return rgb_array
  def _termination(self):
    left_state = p.getLinkState(self._dualarm.dualarmUid, self._dualarm.left_eef_index)
    left_eef_pos = left_state[0]
    left_eef_orn = left_state[1]
    right_state = p.getLinkState(self._dualarm.dualarmUid, self._dualarm.right_eef_index)
    right_eef_pos = right_state[0]
    right_eef_orn = right_state[1]

    blockPos, blockOrn = p.getBasePositionAndOrientation(self.blockUid)

    left_dist = np.linalg.norm(abs(np.array(blockPos)-np.array(left_eef_pos)))
    right_dist = np.linalg.norm(abs(np.array(blockPos)-np.array(right_eef_pos)))



    #print("self._envStepCounter")
    #print(self._envStepCounter)
    if (self.terminated or self._envStepCounter > self._maxSteps):
      self._observation = self.getExtendedObservation()
      return True
    if (0.5*left_dist+0.5*right_dist)<0.1:
      return True
    return False
  def _reward(self):
      reward = -1000;
      blockPos, blockOrn = p.getBasePositionAndOrientation(self.blockUid)
      left_state = p.getLinkState(self._dualarm.dualarmUid, self._dualarm.left_eef_index)
      left_eef_pos = left_state[0]
      left_eef_orn = left_state[1]
      right_state = p.getLinkState(self._dualarm.dualarmUid, self._dualarm.right_eef_index)
      right_eef_pos = right_state[0]
      right_eef_orn = right_state[1]

      left_dist = np.linalg.norm(abs(np.array(blockPos)-np.array(left_eef_pos)))
      right_dist = np.linalg.norm(abs(np.array(blockPos)-np.array(right_eef_pos)))

      if (0.5*left_dist+0.5*right_dist)<0.1:
        reward = reward+10000
      else:
        reward = -(0.5*left_dist+0.5*right_dist)

      return reward


  if parse_version(gym.__version__) < parse_version('0.9.6'):
    _render = render
    _reset = reset
    _seed = seed
    _step = step