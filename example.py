"""
Demonstration of the GazeTracking library.
Check the README.md for complete documentation.
"""

import cv2
from gaze_tracking import GazeTracking
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

	
#movement function we would use. These are step functions.
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



def main():
    gaze = GazeTracking()
    webcam = cv2.VideoCapture(0)

    while True:
        # We get a new frame from the webcam
        _, frame = webcam.read()
        # We send this frame to GazeTracking to analyze it
        gaze.refresh(frame)

        frame = gaze.annotated_frame()
        text = ""

        left_pupil = gaze.pupil_left_coords()
        right_pupil = gaze.pupil_right_coords()

        print(left_pupil)
        print(right_pupil)

        if (gaze.is_blinking() or (left_pupil == None and right_pupil==None)):
            text = "Eyes Closed"
            stop()
        elif gaze.is_right():
            text = "Looking right"
            r()
        elif gaze.is_left():
            text = "Looking left"
            l()
        elif gaze.is_center():
            text = "Looking center"
            f()

        cv2.putText(frame, text, (90, 60), cv2.FONT_HERSHEY_DUPLEX, 1.6, (147, 58, 31), 2)

    
        cv2.putText(frame, "Left pupil:  " + str(left_pupil), (90, 130), cv2.FONT_HERSHEY_DUPLEX, 0.9, (147, 58, 31), 1)
        cv2.putText(frame, "Right pupil: " + str(right_pupil), (90, 165), cv2.FONT_HERSHEY_DUPLEX, 0.9, (147, 58, 31), 1)

        cv2.imshow("Demo", frame)

        if cv2.waitKey(1) == 27:
            break

if __name__ == "__main__":
    main()