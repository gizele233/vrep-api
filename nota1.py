import numpy as np

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
c=0
velocidade=1.0
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Failed connecting to remote API server')

returnC, motorEsquerdo = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
returnC, motorDireito = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)

returnC, sensorFront = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor5', sim.simx_opmode_blocking)
returnC, sensorDireito = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor8', sim.simx_opmode_blocking)
returnC, sensorEsquerdo = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor1', sim.simx_opmode_blocking)

returnC, camera = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking)

returnC = sim.simxSetJointTargetVelocity(clientID,motorEsquerdo,2.0,sim.simx_opmode_streaming)
returnC = sim.simxSetJointTargetVelocity(clientID,motorDireito,2.0,sim.simx_opmode_streaming)

returnC,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,sensorFront,sim.simx_opmode_streaming)
returnC,detectionState2,detectedPoint2,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,sensorEsquerdo,sim.simx_opmode_streaming)
returnC,detectionState3,detectedPoint3,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,sensorDireito,sim.simx_opmode_streaming)

returnC,resolution,image = sim.simxGetVisionSensorImage(clientID,camera,1,sim.simx_opmode_streaming)

for i in range(1,201):
    returnC,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,sensorFront,sim.simx_opmode_buffer)
    returnC,detectionState2,detectedPoint2,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,sensorEsquerdo,sim.simx_opmode_buffer)
    returnC,detectionState3,detectedPoint3,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,sensorDireito,sim.simx_opmode_buffer)

    returnC,resolution,image = sim.simxGetVisionSensorImage(clientID,camera,1,sim.simx_opmode_buffer)

    a = np.linalg.norm(detectedPoint)

    if a>0.01:
        print("Avancar")
        returnC = sim.simxSetJointTargetVelocity(clientID,motorEsquerdo,velocidade,sim.simx_opmode_blocking)
        returnC = sim.simxSetJointTargetVelocity(clientID,motorDireito,velocidade,sim.simx_opmode_blocking)
    if (0.3<a and a<0.75):
        print("Girar")

        #for j in range(1,2):
        returnC = sim.simxSetJointTargetVelocity(clientID,motorDireito,-velocidade,sim.simx_opmode_blocking)
        returnC = sim.simxSetJointTargetVelocity(clientID,motorEsquerdo,velocidade,sim.simx_opmode_blocking)
    
    else:
        returnC = sim.simxSetJointTargetVelocity(clientID,motorEsquerdo,velocidade,sim.simx_opmode_blocking)
        returnC = sim.simxSetJointTargetVelocity(clientID,motorDireito,velocidade,sim.simx_opmode_blocking)
    
    print(a)
    time.sleep(1)

sim.simxFinish(-1)
print("Programa finalizado")
