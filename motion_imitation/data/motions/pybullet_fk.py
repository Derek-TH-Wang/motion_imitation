import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
plane = p.loadURDF("plane.urdf")
p.setGravity(0,0,-9.8)
quadruped = p.loadURDF("laikago/laikago_toes.urdf",[0,0,.5],[0,0.5,0.5,0], useFixedBase=False)


jointIds=[]
paramIds=[]
jointOffsets=[]
jointDirections=[-1,1,1,1,1,1,-1,1,1,1,1,1]
jointAngles=[0,0,0,0,0,0,0,0,0,0,0,0]

for i in range (4):
	jointOffsets.append(0)
	jointOffsets.append(-0.7)
	jointOffsets.append(0.7)


		
p.getCameraImage(480,320)
p.setRealTimeSimulation(1)


for :
	
	for i in range(len(paramIds)):

		p.setJointMotorControl2(quadruped,jointIds[i],p.POSITION_CONTROL,jointDirections[i]*targetPos+jointOffsets[i])
	