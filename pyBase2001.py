from hub import light_matrix, motion_sensor, port
from app import display
import runloop
import motor
import motor_pair

motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)

kp = 2
checkNegative = 0


##FUNCTIONS

##Yaw Adjust
def yawAngle(checkNegative):
    yaw = motion_sensor.tilt_angles()[0]
    
    if checkNegative == 1 and yaw < 0:
        return yaw + 3600
    elif checkNegative == -1 and yaw > 0:
        return yaw - 3600
    else:
        return yaw

def resetRelativePos():
    motor.reset_relative_position(port.E,0)
    motor.reset_relative_position(port.A,0)

##Find tDegrees

def tDegrees():
    return (motor.relative_position(port.E) + motor.relative_position(port.A))/2
##GyroFwd

def gyroFwd(degrees, speed, targetAngle,checkNegative):
    motor.reset_relative_position(port.E, 0) ##Reset relative position
    motor.reset_relative_position(port.A, 0)

    while not abs(motor.relative_position(port.E))>=degrees: ## Wait until traveled specified degrees
    ##while abs(tDegrees())<degrees:
        currentAngle = yawAngle(checkNegative) ## Get yaw
        error = currentAngle - targetAngle ## Find error
        proportional = error*kp ## Calculate Proportional
        correction = proportional ## Calculate Correction
        motor_pair.move_tank(motor_pair.PAIR_1, speed+correction, speed-correction) ##Set speed

##def gyroRmpFwd(degrees, speed1, speed2, targetAngle, checkNegative):
    ##while not abs

##Yaw Turn

def yawTurn(speed1, speed2, speed3, angle, motorSplit, checkNegative):
    yawStart = yawAngle(1)
    speed = speed1
    speedDiff1 = speed1 - speed2
    speedDiff2 = speed2 - speed3
    error = angle-yawAngle(checkNegative)
    halfYaw = error/2
    while not abs(error) < abs(halfYaw):
        error = angle-yawAngle(checkNegative)
        if abs(error) < 10:
            speed = speed1+(speedDiff1*((yawAngle(checkNegative) - yawStart)/(halfYaw)))
        else:
            speed = speed3+(speedDiff2*((angle - yawAngle(checkNegative))/(halfYaw)))
    if speed2 * angle > 0:
        motor_pair.move_tank(motor_pair.PAIR_1, speed, 0 - speed * motorSplit)
    else:
        motor_pair.move_tank(motor_pair.PAIR_1,  0 - speed * motorSplit, speed1)

async def main():
    # write your code here
    gyroFwd(600, 1000, 0, 0) ##Start
    gyroFwd(500, 1000, 900, 0)
    gyroFwd(500,1000,1800, 1)
    gyroFwd(500,1000, -900, -1)
    gyroFwd(500, 1000, 0, 0)
    motor_pair.stop(motor_pair.PAIR_1)

    yawTurn(1000,1000,1000, -900, 1, 0) ##Turn to grab RSMP


    '''
    gyroFwd(200, 1000, -90)
    gyroFwd(200, 500, -90)
    checkNegative = -1
    gyroFwd(1000, 1000, -1650) ##Send RSMP
    gyroFwd(200, 1000, -1800)
    yawTurn(1000,1000,1000, 0, 1)
    checkNegative = 0
    gyroFwd(1500, 1000, 0) ##Send Drone
    motor_pair.stop(motor_pair.PAIR_1)
    '''

runloop.run(main())


