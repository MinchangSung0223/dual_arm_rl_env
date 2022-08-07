import pybullet as p
import pybullet_data
import time
import numpy as np
import math
import threading


pi = np.pi
ll = [0,-175/180*pi,-175/180*pi,-175/180*pi,-175/180*pi,-175/180*pi,-215/180*pi,0]
ul = [0,175/180*pi,175/180*pi,175/180*pi,175/180*pi,175/180*pi,215/180*pi,0]
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1,0.1]
jr = [0, 6.10865, 6.10865, 6.10865, 6.10865, 6.10865, 6.10865]
def getCameraImage(cam_pos,cam_orn):
	width = 320
	height =240
	near = 0.01
	far = 1000
	fov = 60
	aspect = width/height
	angle = 0.0;
	q = p.getQuaternionFromEuler(cam_orn)
	cam_orn = np.reshape(p.getMatrixFromQuaternion(q ),(3,3));
	cam_orn = [cam_orn[0]-1.5707,cam_orn[1],cam_orn[2]+3.141592]
	view_pos = np.matmul(cam_orn,np.array([-0.001,0,0.0]).T)
	view_pos = np.array(view_pos+cam_pos)
	view_matrix = p.computeViewMatrix([cam_pos[0],cam_pos[1],cam_pos[2]], view_pos, [0,0,1])
	projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	images = p.getCameraImage(width,
					height,
					view_matrix,
					projection_matrix,
					shadow=False)
	return images
def getImageTask():
	while 1:
		image = getCameraImage(camera_T[0],p.getEulerFromQuaternion(camera_T[1]))
		time.sleep(1./60.)

if __name__ == "__main__":
	p.connect(p.GUI)
	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	p.loadURDF("plane.urdf", [0, 0, 0.0])
	dual_armId = p.loadURDF("indy_dual_arm_description/indy_dual_arm.urdf", [0, 0, 0.5])
	left_eef = p.getLinkState(dual_armId,8)
	J = p.getNumJoints(dual_armId);
	for i in range(0,J):
		print(i,p.getJointInfo(dual_armId,i))
	left_joint_list = [2,3,4,5,6,7];
	right_joint_list = [15,16,17,18,19,20];
	joint_list = [left_joint_list,right_joint_list];
	left_joint_states = [0,0,0,0,0,0];
	right_joint_states = [1.5707,0,0,0,0,0];
	joint_states = [0,0,0,0,0,0,0,0,0,0,0,0];
	for i in range(len(left_joint_list)):
			p.resetJointState(dual_armId, left_joint_list[i], left_joint_states[i]);
			p.resetJointState(dual_armId, right_joint_list[i], right_joint_states[i]);

	orn = p.getQuaternionFromEuler([0, -pi, 0])
	camera_T = p.getLinkState(dual_armId,27)

	t = 0;
	t_step =1/240;
	p.setTimeStep(t_step)
	p.setRealTimeSimulation(0)

	count = 0;

	thread1 = threading.Thread(target=getImageTask)
	thread1.start()
	p.setGravity(0,0,-10)
	while 1:
		
		left_pos = [0.7,0.3,0.7]
		right_pos = [0.7,-0.3,0.7]		
		ik_left_joint_states = p.calculateInverseKinematics(dual_armId, 8, left_pos, orn, ll, ul,jr, joint_states)
		ik_right_joint_states = p.calculateInverseKinematics(dual_armId, 21, right_pos, orn, ll, ul,jr, joint_states)
		left_joint_states = ik_left_joint_states[0:6]
		right_joint_states = ik_right_joint_states[10:10+6]
		for i in range(len(left_joint_list)):
			p.resetJointState(dual_armId, left_joint_list[i], left_joint_states[i]);
			p.resetJointState(dual_armId, right_joint_list[i], right_joint_states[i]);
		joint_states =[left_joint_states,ik_right_joint_states] 
		left_eef = p.getLinkState(dual_armId,8)
		right_eef = p.getLinkState(dual_armId,21)
		print(joint_states)
		#p.addUserDebugLine(left_eef[0], np.array(left_eef[0])+np.array([0.1,0,0]), [1, 0, 0], 5,t_step)
		#p.addUserDebugLine(left_eef[0], np.array(left_eef[0])+np.array([0,0.1,0]), [0, 1, 0], 5,t_step)
		#p.addUserDebugLine(left_eef[0], np.array(left_eef[0])+np.array([0,0,0.1]), [0, 0, 1], 5,t_step)
		#p.addUserDebugLine(right_eef[0], np.array(right_eef[0])+0.001, [0, 0, 1], 5,0)


		#print(joint_states)
		
		t = t+t_step;
		count=count+1;
		p.stepSimulation();
		time.sleep(1./240.)
