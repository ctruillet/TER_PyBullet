import pybullet as p
import time
import pybullet_data
from math import pi
from numpy import cos, sin, sqrt, array, absolute, argmin, arctan2

if __name__ == '__main__':
	physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
	p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
	p.setGravity(0, 0, -9.98)
	planeId = p.loadURDF("plane.urdf")
	startPos = [0, 0, 1]
	startOrientation = p.getQuaternionFromEuler([0, 0, 0])

	p.resetDebugVisualizerCamera(cameraDistance=40, cameraYaw=20,cameraPitch=-20, cameraTargetPosition=[0,0,10])

	structure = p.loadURDF("structure_lego.urdf", startPos, startOrientation)

	q1ReelDisplay = p.addUserDebugText("q1 = 0", [0, 0, 2], [255, 0, 0], parentObjectUniqueId=structure,
									   parentLinkIndex=0)
	q2ReelDisplay = p.addUserDebugText("q2 = 0", [0, 0, 2], [255, 0, 0], parentObjectUniqueId=structure,
									   parentLinkIndex=1)
	q3ReelDisplay = p.addUserDebugText("q3 = 0", [0, 0, 2], [255, 0, 0], parentObjectUniqueId=structure,
									   parentLinkIndex=2)

	# Parametres
	q1ParamSlider = p.addUserDebugParameter("q1", -pi, pi, 0)
	q2ParamSlider = p.addUserDebugParameter("q2", -pi, pi, 0)
	q3ParamSlider = p.addUserDebugParameter("q3", -pi, pi, 0)

	while (1):
		p.stepSimulation()
		time.sleep(1. / 240.)
		cubePos, cubeOrn = p.getBasePositionAndOrientation(structure)

		# Recuperation des valeurs des sliders
		q1Param = p.readUserDebugParameter(q1ParamSlider)
		q2Param = p.readUserDebugParameter(q2ParamSlider)
		q3Param = p.readUserDebugParameter(q3ParamSlider)

		# Rotation des rotoïdes
		p.setJointMotorControl2(structure, 0, p.POSITION_CONTROL, targetPosition=q1Param)
		p.setJointMotorControl2(structure, 1, p.POSITION_CONTROL, targetPosition=q2Param)
		p.setJointMotorControl2(structure, 2, p.POSITION_CONTROL, targetPosition=q3Param)

		# Recuperation des valeurs réels des qi
		q1 = p.getJointState(structure, 0)[0]
		q2 = p.getJointState(structure, 1)[0]
		q3 = p.getJointState(structure, 2)[0]

		# Afficher qi réel
		q1ReelDisplay = p.addUserDebugText("q1 = " + str(round(q1, 3)), [0, 0, 2], [255, 0, 0],
										   parentObjectUniqueId=structure, parentLinkIndex=0,
										   replaceItemUniqueId=q1ReelDisplay)
		q2ReelDisplay = p.addUserDebugText("q2 = " + str(round(q2, 3)), [-.5, 0, 0.2], [255, 0, 0],
										   parentObjectUniqueId=structure, parentLinkIndex=1,
										   replaceItemUniqueId=q2ReelDisplay)
		q3ReelDisplay = p.addUserDebugText("q3 = " + str(round(q3, 3)), [0, 0, 0.2], [255, 0, 0],
										   parentObjectUniqueId=structure, parentLinkIndex=2,
										   replaceItemUniqueId=q3ReelDisplay)


	p.disconnect()
