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
    pass

  def reset(self):
      pass


  def getActionDimension(self):
        pass

  def getObservationDimension(self):
    pass


  def getObservation(self):
    pass


  def applyAction(self, motorCommands):
    pass
