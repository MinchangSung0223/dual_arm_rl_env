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
    self.left_joint_states = [0.1589816820093537, -1.2737553471542125, -0.6723662360600272, 1.1190622645438093, -1.2547184558128601, -0.25946008529603365];    
    self.right_joint_states = [3.1503967734590415, -1.2634881303083036, -1.5956838806799498, -1.4029169666936954, -1.072274654110312, 1.2478745720284445];
    self.left_gripper_joint_states = [0,0,0,0];    
    self.right_gripper_joint_states = [0,0,0,0];       
    self.left_eef_index =  8
    self.right_eef_index =  21
    self.camera_index =  27
    
    self.numJoints = 12
    self.kukaEndEffectorIndex = 6
    self.kukaGripperIndex = 7
    self.ll = [-1.527162778,-1.527162778, -3.141592,-3.141592,-3.141592]
    #upper limits for null space
    self.ul = [1.527162778,1.527162778, 3.141592,3.141592,3.141592]
    #joint ranges for null space
    self.jr = [0,0,0,0,0,0]
    #restposes for null space
    self.rp = [0,0,0,0,0,0]
    #joint damping coefficents
    self.jd = [
        0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001
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
        p.resetJointState(self.dualarmUid, jointIndex, self.jointPositions[jointIndex])
        p.setJointMotorControl2(self.dualarmUid,
                              jointIndex,
                              p.POSITION_CONTROL,
                              targetPosition=self.jointPositions[jointIndex],
                              force=self.maxForce)
    self.left_endEffectorPos = [0.7, 0.3, 0.7]
    self.right_endEffectorPos = [0.7, -0.3, 0.7]
    self.left_endEffectorAngle=0
    self.right_endEffectorAngle=0
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
      if (self.useInverseKinematics):
        return len(self.motorIndices)
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
    if (self.useInverseKinematics):
      left_dx = motorCommands[0]
      left_dy = motorCommands[1]
      left_dz = motorCommands[2]
      left_da = motorCommands[3]
      left_fingerAngle = motorCommands[4]
      right_dx = motorCommands[5]
      right_dy = motorCommands[6]
      right_dz = motorCommands[7]
      right_da = motorCommands[8]
      right_fingerAngle = motorCommands[9]      

      state = p.getLinkState(self.dualarmUid, self.left_eef_index)
      actual_left_endEffectorPos = state[0]
      #print("pos[2] (getLinkState(kukaEndEffectorIndex)")
      #print(actualEndEffectorPos[2])

      self.left_endEffectorPos[0] = self.left_endEffectorPos[0] + left_dx
      if (self.left_endEffectorPos[0] > 0.7):
        self.left_endEffectorPos[0] = 0.7
      if (self.left_endEffectorPos[0] < 0.5):
        self.left_endEffectorPos[0] = 0.50
      self.left_endEffectorPos[1] = self.left_endEffectorPos[1] + left_dy
      if (self.left_endEffectorPos[1] < -0.3):
        self.left_endEffectorPos[1] = -0.3
      if (self.left_endEffectorPos[1] > 0.3):
        self.left_endEffectorPos[1] = 0.3
      self.left_endEffectorPos[2] = self.left_endEffectorPos[2] + left_dz
      if (self.left_endEffectorPos[1] < 0.7):
        self.left_endEffectorPos[1] = 0.7
      if (self.left_endEffectorPos[1] > 0.5):
        self.left_endEffectorPos[1] = 0.5


      self.left_endEffectorAngle = self.left_endEffectorAngle + left_da
      pos = self.left_endEffectorPos
      orn = p.getQuaternionFromEuler([0, -math.pi, 0])  # -math.pi,yaw])

      jointPoses = p.calculateInverseKinematics(self.dualarmUid, self.left_eef_index, pos,
                                                    orn, self.ll, self.ul, self.jr, self.rp)

      #print("jointPoses")
      #print(jointPoses)
      #print("self.kukaEndEffectorIndex")
      #print(self.kukaEndEffectorIndex)
      if (self.useSimulation):
        for i in range(self.left_eef_index+1):
          #print(i)
          p.setJointMotorControl2(bodyUniqueId=self.dualarmUid,
                                  jointIndex=i,
                                  controlMode=p.POSITION_CONTROL,
                                  targetPosition=jointPoses[i],
                                  targetVelocity=0,
                                  force=self.maxForce,
                                  maxVelocity=self.maxVelocity,
                                  positionGain=0.3,
                                  velocityGain=1)
      else:
        #reset the joint state (ignoring all dynamics, not recommended to use during simulation)
        for i in range(self.numJoints):
          p.resetJointState(self.dualarmUid, i, jointPoses[i])
    else:
      for action in range(len(motorCommands)):
        motor = self.motorIndices[action]
        p.setJointMotorControl2(self.dualarmUid,
                                motor,
                                p.POSITION_CONTROL,
                                targetPosition=motorCommands[action],
                                force=self.maxForce)