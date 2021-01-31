
import pybullet as p
import pybullet_data

p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#Loading plane in the simulator
p.loadURDF("plane.urdf")


p.setGravity(0, 0, -10)

carpos = [2, 0, 0.1]




#Loading car at carpos position
car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2]) #husky is urdf file of a car which comes with PyBullet


numJoints = p.getNumJoints(car)
for joint in range(numJoints):
  print(p.getJointInfo(car, joint))


maxForce = 100 #Newton: Max force to be applied to make the car move in the given velocity

def __move(car, leftFrontWheel, rightFrontWheel, leftRearWheel, rightRearWheel):
		p.setJointMotorControl2(car,  4, p.VELOCITY_CONTROL, targetVelocity=leftFrontWheel, force=30)
		p.setJointMotorControl2(car,  5, p.VELOCITY_CONTROL, targetVelocity=rightFrontWheel, force=30)
		p.setJointMotorControl2(car,  6, p.VELOCITY_CONTROL, targetVelocity=leftRearWheel, force=30)
		p.setJointMotorControl2(car,  7, p.VELOCITY_CONTROL, targetVelocity=rightRearWheel, force=30)

while (1):
    keys = p.getKeyboardEvents()
    for k, v in keys.items():
        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = 1
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
           
            p.stepSimulation()
        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()
          
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = -1
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = -0.5
            p.setJointMotorControl2(car, 2, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.setJointMotorControl2(car, 4, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            targetVel = 0.5
            p.setJointMotorControl2(car, 3, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.setJointMotorControl2(car, 5, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()
        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = 0.5
            p.setJointMotorControl2(car, 2, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.setJointMotorControl2(car, 4, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            targetVel = -0.5
            p.setJointMotorControl2(car, 3, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.setJointMotorControl2(car, 5, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()
        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()



p.getContactPoints(car)

p.disconnect()