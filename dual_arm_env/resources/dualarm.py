import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import pybullet as p
import numpy as np
import copy
import math
import pybullet_data


class DualArm:
  def __init__(self, urdfRootPath=pybullet_data.getDataPath(), timeStep=0.01):
    self.urdfRootPath = urdfRootPath
    self.timeStep = timeStep
    self.maxVelocity = 1.3 # 1,2,3 150deg/s 4,5,6 180deg/s
    self.maxForce = 100.
    self.fingerAForce = 2
    self.fingerBForce = 2
    self.fingerTipForce = 2
    self.useInverseKinematics = 1
    self.useSimulation = 1
    self.useNullSpace = 21
    self.useOrientation = 1
    self.left_joint_list = [2,3,4,5,6,7];
    self.right_joint_list = [15,16,17,18,19,20];  
    self.left_gripper_joint_list = [10,11,12,13];    
    self.right_gripper_joint_list = [23,24,25,26];    
    self.left_joint_states = [0,0,0,0,0,0];    
    self.right_joint_states = [0,0,0,0,0,0];
    self.left_gripper_joint_states = [0,0,0,0];    
    self.right_gripper_joint_states = [0,0,0,0];       
    self.left_eef_index =  8
    self.right_eef_index =  21
    self.camera_index =  27
    
    self.numJoints = 12
    self.kukaEndEffectorIndex = 6
    self.kukaGripperIndex = 7
    self.ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
    #upper limits for null space
    self.ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
    #joint ranges for null space
    self.jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
    #restposes for null space
    self.rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
    #joint damping coefficents
    self.jd = [
        0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001,
        0.00001, 0.00001, 0.00001, 0.00001
    ]
    self.reset()
  def reset(self):
    self.dualarmUid= p.loadURDF(os.path.join(self.urdfRootPath, "indy_dual_arm_description/indy_dual_arm.urdf"), [0, 0, 0.5])
    self.jointPositions = [
        0,0,self.left_joint_states[0],self.left_joint_states[1],self.left_joint_states[2],self.left_joint_states[3],self.left_joint_states[4],self.left_joint_states[5],0,0,
        self.left_gripper_joint_states[0],self.left_gripper_joint_states[1],self.left_gripper_joint_states[2],self.left_gripper_joint_states[3],
        0,self.right_joint_states[0],self.right_joint_states[1],self.right_joint_states[2],self.right_joint_states[3],self.right_joint_states[4],self.right_joint_states[5],0,0,
        self.right_gripper_joint_states[0],self.right_gripper_joint_states[1],self.right_gripper_joint_states[2],self.right_gripper_joint_states[3]
    ]
    self.numJoints = p.getNumJoints(self.dualarmUid)
    for jointIndex  in range(self.numJoints-1):
        p.setJointMotorControl2(self.dualarmUid,
                              jointIndex,
                              p.POSITION_CONTROL,
                              targetPosition=self.jointPositions[jointIndex],
                              force=self.maxForce)
    self.left_endEffectorPos = [0.7, 0.3, 0.7]
    self.right_endEffectorPos = [0.7, -0.3, 0.7]
    self.left_endEffectorOrn = [0, -np.pi,0]
    self.right_endEffectorOrn = [0,-np.pi,0]
    self.motorNames = []
    self.motorIndices = []
    for i in range(self.numJoints):
      jointInfo = p.getJointInfo(self.dualarmUid, i)
      qIndex = jointInfo[3]
      if qIndex > -1:
        self.motorNames.append(str(jointInfo[1]))
        self.motorIndices.append(i)
  def getActionDimension(self):
      # left 6  #right 6
      return 12

  def getObservationDimension(self):
    return len(self.getObservation())


  def getObservation(self):
    observation = []
    left_state = p.getLinkState(self.dualarmUid, self.left_eef_index)
    left_pos = left_state[0]
    left_orn = left_state[1]
    right_state = p.getLinkState(self.dualarmUid, self.right_eef_index)
    right_pos = right_state[0]
    right_orn = right_state[1]
    observation.extend(list(left_pos))
    observation.extend(list(left_orn))
    observation.extend(list(right_pos))
    observation.extend(list(right_orn))

    return observation

  def applyAction(self, motorCommands):
      for i in range(len(motorCommands)):
        if i < 6:
          p.setJointMotorControl2(self.dualarmUid,
          self.left_joint_list[i],
          p.POSITION_CONTROL,
          targetPosition=motorCommands[i],
          force=self.maxForce) 
        else:
          p.setJointMotorControl2(self.dualarmUid,
          self.right_joint_list[i-6],
          p.POSITION_CONTROL,
          targetPosition=motorCommands[i],
          force=self.maxForce) 
    
