#!/usr/bin/env python

import roslib
import rospy
from fw_wrapper.srv import *
import time

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
    r = rospy.Rate(10) # 10hz
    
    # motor position check 
    for i in range(1,9):
        motor_check_byid = i
        motor_position = getMotorPositionCommand(motor_check_byid)
        rospy.loginfo("Motor position of motor %d: %f", motor_check_byid, motor_position)
        
    # return to neutral
    pos_list = [510,512,493,510,687,320,516,512]
    for i, pos in enumerate(pos_list):
        setMotorTargetPositionCommand(i+1, pos)
    time.sleep(1)
   
    while not rospy.is_shutdown():
        # call function to get sensor value
        port = 1
        sensor_reading = getSensorValue(port)
        rospy.loginfo("Sensor value at port %d: %f", port, sensor_reading)
        
        # When foreign object is slightly close to bot's sensor
        if sensor_reading > 0 and sensor_reading < 200:
            # return to neutral
            pos_list = [510,512,493,510,687,320,516,512]
            for i, pos in enumerate(pos_list):
                setMotorTargetPositionCommand(i+1, pos)
        # When foreign object is very close to bot's sensor
        elif sensor_reading >= 200:
            #Point
            setMotorTargetPositionCommand(5, 1023)
            setMotorTargetPositionCommand(6, 199)
            setMotorTargetPositionCommand(7, 470)
            setMotorTargetPositionCommand(8, 511)
        #When no object detected
        else:
            # Dance
            
            # Right Elbow
            if getMotorPositionCommand(7) <= 530:
                setMotorTargetPositionCommand(7, 800)
            if getMotorPositionCommand(7) >= 795:
                setMotorTargetPositionCommand(7, 512)
            
            # Left Elbow
            if getMotorPositionCommand(8) <= 235:
                setMotorTargetPositionCommand(8, 512)    
            if getMotorPositionCommand(8) >= 490:
                setMotorTargetPositionCommand(8, 220)
            
            # Right hip
            if getMotorPositionCommand(1) <= 515:
                setMotorTargetPositionCommand(1, 550)    
            if getMotorPositionCommand(1) >= 545: 
                setMotorTargetPositionCommand(1, 510)
            
            # Left hip    
            if getMotorPositionCommand(2) <= 475:
                setMotorTargetPositionCommand(2, 510)    
            if getMotorPositionCommand(2) >= 505: 
                setMotorTargetPositionCommand(2, 470)                
            
            
        # Sleep to enforce loop rate
        r.sleep()
        
        
        
        
        
        
