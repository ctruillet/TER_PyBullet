import pybullet as p
import pybullet_data
from math import pi
import time
import numpy as np
from numpy import cos, sin, sqrt, array, absolute, argmin, arctan2

if __name__ == '__main__':
    ptCible = [-10.37, -10.05, 30.79]

    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    p.setGravity(0, 0, -9.81)
    planeId = p.loadURDF("plane.urdf")
    startPos = [0, 0, 1]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])

    logQi = open("logQi.csv", "w")
    logQi.write("t q1 q2 q3\n")

    p.resetDebugVisualizerCamera(cameraDistance=40, cameraYaw=20, cameraPitch=-20, cameraTargetPosition=[0, 0, 10])

    structure = p.loadURDF("structure_lego.urdf", startPos, startOrientation)

    q1ReelDisplay = p.addUserDebugText("q1 = 0", [0, 0, 2], [255, 0, 0], parentObjectUniqueId=structure,
                                       parentLinkIndex=0)
    q2ReelDisplay = p.addUserDebugText("q2 = 0", [0, 0, 2], [255, 0, 0], parentObjectUniqueId=structure,
                                       parentLinkIndex=1)
    q3ReelDisplay = p.addUserDebugText("q3 = 0", [0, 0, 2], [255, 0, 0], parentObjectUniqueId=structure,
                                       parentLinkIndex=2)

    positionOT = np.around(p.getLinkState(structure, 4)[0], 2)
    OTDisplay = p.addUserDebugText("OT", positionOT, textColorRGB=[0, 0, 0])

    # Parametres
    q1ParamSlider = p.addUserDebugParameter("q1", -pi, pi, 0)
    q2ParamSlider = p.addUserDebugParameter("q2", -pi, pi, 0)
    q3ParamSlider = p.addUserDebugParameter("q3", -pi, pi, 0)

    t0 = time.perf_counter()
    while (1):
        p.stepSimulation()
        time.sleep(1. / 240.)
        cubePos, cubeOrn = p.getBasePositionAndOrientation(structure)

        # Recuperation des valeurs des sliders
        q1Param = p.readUserDebugParameter(q1ParamSlider)
        q2Param = p.readUserDebugParameter(q2ParamSlider)
        q3Param = p.readUserDebugParameter(q3ParamSlider)

        # Recuperation de la position de l'OT
        positionOT = np.around(p.getLinkState(structure, 4)[0], 2)
        OTDisplay = p.addUserDebugText("OT  = " + str(positionOT), p.getLinkState(structure, 4)[0],
                                       textColorRGB=[255, 0, 0],
                                       replaceItemUniqueId=OTDisplay)

        # Rotation des rotoïdes
        if time.perf_counter() - t0 < 3:
            p.setJointMotorControl2(structure, 0, p.POSITION_CONTROL, targetPosition=q1Param)
            p.setJointMotorControl2(structure, 1, p.POSITION_CONTROL, targetPosition=q2Param)
            p.setJointMotorControl2(structure, 2, p.POSITION_CONTROL, targetPosition=q3Param)

        else:
            jointPoses = p.calculateInverseKinematics(structure, 4, ptCible)
            p.setJointMotorControl2(structure, 0, p.POSITION_CONTROL, targetPosition=jointPoses[0])
            p.setJointMotorControl2(structure, 1, p.POSITION_CONTROL, targetPosition=jointPoses[1])
            p.setJointMotorControl2(structure, 2, p.POSITION_CONTROL, targetPosition=jointPoses[2])

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

        logQi.write(
            str(round(time.perf_counter() - t0, 3)) + " " + str(round(q1, 3)) + " " + str(round(q2, 3)) + " " + str(
                round(q3, 3)) + "\n")

    p.disconnect()
