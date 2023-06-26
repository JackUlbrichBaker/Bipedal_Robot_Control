#!/usr/bin/env python
# coding: utf-8

# In[ ]:





# In[52]:


import pybullet as p
import pybullet_data
import os
import time

import pybullet as p
import pybullet_data as pd
import math
import time

p.connect(p.GUI, options="--opengl2")
p.setAdditionalSearchPath(pd.getDataPath())
base_path = pd.getDataPath()
print(pd.getDataPath())


# In[62]:


p.resetSimulation()

p.setGravity(0, 0, -10)
planeId = p.loadURDF(os.path.join(base_path, "plane.urdf"))
cubeStartPos = [0, 0, .5]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
bip_path = "bip/Bip.SLDASM/urdf/Bip.SLDASM.urdf"
boxId = p.loadURDF(bip_path, cubeStartPos, cubeStartOrientation, useFixedBase = False)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

useRealTimeSimulation = 0


# In[63]:


def motor_control_pos(motors, Pos, robot):
    # Input:  motors n x 1 matrix with motor index (int)
    #         Pos    n x 1 matrix with desired motor position (float)
    
    Maxforce = 500
    p.setJointMotorControlArray(robot, motors, targetPositions = Pos, controlMode=p.POSITION_CONTROL)
    


# In[64]:


for i in range (100):
    p.stepSimulation()
    time.sleep(1./24)
    target = 30
    pos = target/100*i
    motor_control_pos([0],[pos], boxId)
    
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)


# In[50]:


import numpy as np
from PIL import Image
from IPython.display import display

width = 320
height = 200
img_arr = p.getCameraImage(
    width,
    height,
    viewMatrix=p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=cubePos,
        distance=.5,
        yaw=60,
        pitch=-10,
        roll=0,
        upAxisIndex=2,
    ),
    projectionMatrix=p.computeProjectionMatrixFOV(
        fov=60,
        aspect=width/height,
        nearVal=0.01,
        farVal=100,
    ),
    shadow=True,
    lightDirection=[1, 1, 1],
)

width, height, rgba, depth, mask = img_arr
print(f"rgba shape={rgba.shape}, dtype={rgba.dtype}")
display(Image.fromarray(rgba, 'RGBA'))
print(f"depth shape={depth.shape}, dtype={depth.dtype}, as values from 0.0 (near) to 1.0 (far)")
display(Image.fromarray((depth*255).astype('uint8')))
print(f"mask shape={mask.shape}, dtype={mask.dtype}, as unique values from 0 to N-1 entities, and -1 as None")
display(Image.fromarray(np.interp(mask, (-1, mask.max()), (0, 255)).astype('uint8')))


# In[51]:


p.disconnect()


# In[ ]:




