
import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#Loading plane in the simulator
p.loadURDF("plane.urdf")


p.setGravity(0, 0, -10)

carpos = [0, 0, 0]




#Loading car at carpos position
car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2]) #husky is urdf file of a car which comes with PyBullet


numJoints = p.getNumJoints(car)
for joint in range(numJoints):
  print(p.getJointInfo(car, joint))


maxForce = 30 #Newton: Max force to be applied to make the car move in the given velocity


def forward():
	targetVel = 1
	for joint in range(2, 6):
		p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
	p.stepSimulation()

def backward():
	targetVel = -1
	for joint in range(2, 6):
		p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
	p.stepSimulation()

def stop():
	targetVel = 0
	for joint in range(2, 6):
		p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
	p.stepSimulation()

def left():
	targetVel = -0.5
	p.setJointMotorControl2(car, 2, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
	p.setJointMotorControl2(car, 4, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
	targetVel = 0.5
	p.setJointMotorControl2(car, 3, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
	p.setJointMotorControl2(car, 5, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
	p.stepSimulation()

def right():
	targetVel = 0.5
	p.setJointMotorControl2(car, 2, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
	p.setJointMotorControl2(car, 4, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
	targetVel = -0.5
	p.setJointMotorControl2(car, 3, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
	p.setJointMotorControl2(car, 5, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
	p.stepSimulation()

def f():
	for i in range(800):
		forward()
	stop()
def b():
	for i in range(800):
		backward()
	stop()

def l():
	for i in range(800):
		left()
	stop()

def r():
	for i in range(800):
		right()
	stop()


#main code

time.sleep(0.5)
f()
f()
f()
b()
b()
b()
l()
l()
l()
l()
r()
r()
r()
r()
r()
r()
r()
r()
l()
l()
l()
l()
time.sleep(1)

p.getContactPoints(car)

p.disconnect()
