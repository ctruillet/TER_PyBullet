import pybullet as p
import pybullet_data
from math import *
from math import pi, inf
import time
import numpy as np
from pdControllerStable import PDControllerStable

if __name__ == '__main__':
    ptCible = [-100, -100, 200]

    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    pd = p.loadPlugin("pdControlPlugin")

    # PD CONTROL
    kp = 125
    kd = 10
    sPD = PDControllerStable(p)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    p.setGravity(0, 0, -9.81)
    planeId = p.loadURDF("plane.urdf")
    startPos = [0, 0, 1]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])

    logQi = open("logQi.csv", "w")
    logQi.write("t q1 q2 q3\n")

    p.resetDebugVisualizerCamera(cameraDistance=50, cameraYaw=180, cameraPitch=-20, cameraTargetPosition=[0, 0, 10])

    structure = p.loadURDF("structure_lego.urdf", startPos, startOrientation)
    p.changeDynamics(structure, -1, mass=0.572)
    p.changeDynamics(structure, 0, mass=0.096)
    p.changeDynamics(structure, 1, mass=0.097)
    p.changeDynamics(structure, 3, mass=0.011)

    q1ReelDisplay = p.addUserDebugText("q1 = 0", [0, 0, 2], [255, 0, 0], parentObjectUniqueId=structure,
                                       parentLinkIndex=0)
    q2ReelDisplay = p.addUserDebugText("q2 = 0", [0, 0, 2], [255, 0, 0], parentObjectUniqueId=structure,
                                       parentLinkIndex=1)
    q3ReelDisplay = p.addUserDebugText("q3 = 0", [0, 0, 2], [255, 0, 0], parentObjectUniqueId=structure,
                                       parentLinkIndex=2)

    positionOriginArm = np.around(p.getLinkState(structure, 2)[0], 2)
    LineDisplay = p.addUserDebugLine(positionOriginArm, ptCible, [0, 255, 0], lineWidth=0.1, lifeTime=1. / 240.)

    p.addUserDebugText("o - ptCible", ptCible, [255, 0, 0], lifeTime=0)

    positionOT = np.around(p.getLinkState(structure, 4)[0], 2)
    OTDisplay = p.addUserDebugText("OT", positionOT, textColorRGB=[0, 0, 0])

    # Parametres
    angleCameraYawSlider = p.addUserDebugParameter("Camera Yaw", 0, 360, 180)
    angleCameraPitchSlider = p.addUserDebugParameter("Camera Pitch", -180, 180, -20)
    angleCameraDistanceSlider = p.addUserDebugParameter("Camera Distance", 0, 200, 50)

    rKey = ord('r')
    keys = p.getKeyboardEvents()

    # q1ParamSlider = p.addUserDebugParameter("q1", -pi, pi, 0)
    # q2ParamSlider = p.addUserDebugParameter("q2", -pi, pi, 0)
    # q3ParamSlider = p.addUserDebugParameter("q3", -pi, pi, 0)

    t0 = time.perf_counter()
    Qdebut = [p.getJointState(structure, 0)[0],
              p.getJointState(structure, 1)[0],
              p.getJointState(structure, 2)[0]]

    # Calcul du pt Atteignable associé
    c = 13
    D = np.linalg.norm(np.array(ptCible) - np.array(positionOriginArm))
    ptCible2 = [positionOriginArm[0] + c * (ptCible[0] - positionOriginArm[0]) / D,
                positionOriginArm[1] + c * (ptCible[1] - positionOriginArm[1]) / D,
                positionOriginArm[2] + c * (ptCible[2] - positionOriginArm[2]) / D]

    # print(ptCible2)
    while 1:
        p.stepSimulation()
        p.getCameraImage(320, 200)

        time.sleep(1. / 240.)
        cubePos, cubeOrn = p.getBasePositionAndOrientation(structure)

        # Recuperation des valeurs des sliders
        angleCameraYaw = p.readUserDebugParameter(angleCameraYawSlider)
        angleCameraDistance = p.readUserDebugParameter(angleCameraDistanceSlider)
        angleCameraPitch = p.readUserDebugParameter(angleCameraPitchSlider)

        p.resetDebugVisualizerCamera(cameraDistance=angleCameraDistance,
                                     cameraYaw=angleCameraYaw,
                                     cameraPitch=angleCameraPitch,
                                     cameraTargetPosition=[0, 0, 10])
        # q1Param = p.readUserDebugParameter(q1ParamSlider)
        # q2Param = p.readUserDebugParameter(q2ParamSlider)
        # q3Param = p.readUserDebugParameter(q3ParamSlider)

        # Recuperation de la position de l'OT
        positionOT = np.around(p.getLinkState(structure, 4)[0], 2)
        OTDisplay = p.addUserDebugText("OT  = " + str(positionOT), p.getLinkState(structure, 4)[0],
                                       textColorRGB=[255, 0, 0],
                                       replaceItemUniqueId=OTDisplay)
        positionOriginArm = np.around(p.getLinkState(structure, 2)[0], 2)
        LineDisplay = p.addUserDebugLine(positionOriginArm,
                                         ptCible,
                                         lineColorRGB=[0, 255, 0],
                                         lineWidth=0.1,
                                         lifeTime=0.05)

        # RESET
        keys = p.getKeyboardEvents()
        if rKey in keys and keys[rKey] & p.KEY_WAS_TRIGGERED:
            p.resetJointState(structure, 0, Qdebut[0])
            p.resetJointState(structure, 1, Qdebut[1])
            p.resetJointState(structure, 2, Qdebut[2])

            t0 = time.perf_counter()

        # print(t0, "(", time.perf_counter() - t0 ,")")

        # Rotation des rotoïdes
        if time.perf_counter() - t0 < 2:
            pass
            # p.setJointMotorControl2(structure, 0, p.PD_CONTROL, targetPosition=q1Param)
            # p.setJointMotorControl2(structure, 1, p.PD_CONTROL, targetPosition=q2Param)
            # p.setJointMotorControl2(structure, 2, p.PD_CONTROL, targetPosition=q3Param)

        else:
            D = np.linalg.norm(np.array(ptCible) - np.array(positionOriginArm))
            ptCible2 = [positionOriginArm[0] + c * (ptCible[0] - positionOriginArm[0]) / D,
                        positionOriginArm[1] + c * (ptCible[1] - positionOriginArm[1]) / D,
                        positionOriginArm[2] + c * (ptCible[2] - positionOriginArm[2]) / D]

            jointPoses = p.calculateInverseKinematics(structure, 4, ptCible2)
            #jointPoses = [QCible[0], QCible[1], QCible[2]]
            # taus = sPD.computePD(structure, [0, 1, 2], jointPoses,
            #                      [10, 10, 10], [kp, kp, kp], [kd, kd, kd],
            #                      [100, 100, 100], 1. / 240.)
            # p.setJointMotorControlArray(structure, [0, 1, 2], controlMode=p.TORQUE_CONTROL, forces=taus)

            p.setJointMotorControl2(structure,
                                    0,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[0])
            p.setJointMotorControl2(structure,
                                    1,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[1])
            p.setJointMotorControl2(structure,
                                    2,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[2])



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
        logQi.write(str(round(time.perf_counter() - t0, 3)) +
                    " " + str(round(q1, 3)) +
                    " " + str(round(q2, 3)) +
                    " " + str(round(q3, 3)) + "\n")

    p.disconnect()
