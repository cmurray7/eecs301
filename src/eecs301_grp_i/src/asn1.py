#!/usr/bin/env python

import roslib
import rospy
from fw_wrapper.srv import *
import time
import sys

# -----------SERVICE DEFINITION-----------
# allcmd REQUEST DATA
# ---------
# string command_type
# int8 device_id
# int16 target_val
# int8 n_dev
# int8[] dev_ids
# int16[] target_vals

# allcmd RESPONSE DATA
# ---------
# int16 val
# --------END SERVICE DEFINITION----------

# ----------COMMAND TYPE LIST-------------
# GetMotorTargetPosition
# GetMotorCurrentPosition
# GetIsMotorMoving
# GetSensorValue
# GetMotorWheelSpeed
# SetMotorTargetPosition
# SetMotorTargetSpeed
# SetMotorTargetPositionsSync
# SetMotorMode
# SetMotorWheelSpeed

# wrapper function to call service to set a motor mode
# 0 = set target positions, 1 = set wheel moving
def setMotorMode(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorMode', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get motor wheel speed
def getMotorWheelSpeed(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorWheelSpeed', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor wheel speed
def setMotorWheelSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorWheelSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor target speed
def setMotorTargetSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get sensor value
def getSensorValue(port):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetSensorValue', port, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set a motor target position
def setMotorTargetPositionCommand(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
	resp1 = send_command('SetMotorTargetPosition', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get a motor's current position
def getMotorPositionCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
	resp1 = send_command('GetMotorCurrentPosition', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to check if a motor is currently moving
def getIsMotorMovingCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
	resp1 = send_command('GetIsMotorMoving', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")

    # control loop running at 10hz
    r = rospy.Rate(1)# 10hz
        
    # return to neutral
    pos_list = [510,510,510,510,772,250,512,512]
    for i, pos in enumerate(pos_list):
        setMotorTargetPositionCommand(i+1, pos)
    time.sleep(1)
    
    # motor position check 
    for i in range(1,9):
        motor_check_byid = i
        motor_position = getMotorPositionCommand(motor_check_byid)
        rospy.loginfo("Motor position of motor %d: %f", motor_check_byid, motor_position)
        setMotorWheelSpeed(motor_check_byid, 200)
  
    '''
    # Right turn
    setMotorTargetPositionCommand(3, 680)
    time.sleep(0.1)
    setMotorTargetPositionCommand(4, 610)
    time.sleep(0.1)
    setMotorTargetPositionCommand(1, 810)
    setMotorTargetPositionCommand(3, 510)
    time.sleep(0.1)
    setMotorTargetPositionCommand(2, 380)
    setMotorTargetPositionCommand(4, 550)
    time.sleep(0.2)
 #   setMotorTargetPositionCommand(3, 530)
    '''
    
    setMotorTargetPositionCommand(4, 370)
    setMotorTargetPositionCommand(3, 420)
    time.sleep(0)
    setMotorTargetPositionCommand(1, 400)
    setMotorTargetPositionCommand(2, 400)
    time.sleep(1)
    setMotorTargetPositionCommand(1, 650)
    time.sleep(2)
    setMotorTargetPositionCommand(3, 510)
    setMotorTargetPositionCommand(4, 510)
    setMotorTargetPositionCommand(1, 510)
    setMotorTargetPositionCommand(2, 510)



  
    '''  
    while not rospy.is_shutdown():

        # call function to get sensor value
        port = 1
        sensor_reading = getSensorValue(port)
        rospy.loginfo("Sensor value at port %d: %f", port, sensor_reading)       

        # Right-step
        setMotorTargetPositionCommand(4, 370)
        setMotorTargetPositionCommand(3, 420)
        time.sleep(0)
        setMotorTargetPositionCommand(1, 400)
        setMotorTargetPositionCommand(2, 400)
        time.sleep(0)
        setMotorTargetPositionCommand(3, 510)
        setMotorTargetPositionCommand(4, 510)
        setMotorTargetPositionCommand(1, 510)
        setMotorTargetPositionCommand(2, 510)
        time.sleep(0)
        # Left-step      
        setMotorTargetPositionCommand(3, 650)
        setMotorTargetPositionCommand(4, 600)  
        time.sleep(0) 
        setMotorTargetPositionCommand(2, 620)
        setMotorTargetPositionCommand(1, 620)
        time.sleep(0)
        setMotorTargetPositionCommand(4, 510)
        setMotorTargetPositionCommand(3, 510)
        setMotorTargetPositionCommand(2, 510)
        setMotorTargetPositionCommand(1, 510)
        time.sleep(0)
           
            
        # Sleep to enforce loop rate
        r.sleep()
     '''
        
        
        
