from hub import light_matrix, motion_sensor, port
from app import display
import runloop
import motor
import motor_pair

motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)

kplf=0.5
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

## reset relative pos
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


##gyro rmp fwd
def gyroRmpFwd(degrees, speed1, speed2, targetAngle, checkNegative):
    motor.reset_relative_position(port.E, 0) ##Reset relative position
    motor.reset_relative_position(port.A, 0)

    speedDiff=speed2-speed1
    while not abs(motor.relative_position(port.E))>=degrees: ## Wait until traveled specified degrees
    ##while abs(tDegrees())<degrees:
        currentAngle = yawAngle(checkNegative) ## Get yaw
        error = currentAngle - targetAngle ## Find error
        proportional = error*kp ## Calculate Proportional
        correction = proportional ## Calculate Correction
        speed = speed1+(speedDiff*(motor.relative_position(port.E)/degrees))
        motor_pair.move_tank(motor_pair.PAIR_1, speed+correction, speed-correction) ##Set speed

##Yaw Turn
'''

def yawTurn(speed1, speed2, speed3, angle, motorSplit, checkNegative):
    yawStart = yawAngle(1)
    speed = speed1
    speedDiff1 = speed2 - speed1
    speedDiff2 = speed2 - speed3
    error = angle-yawAngle(checkNegative)
    halfYaw = abs(error)/2
    while not abs(error) < 10 :
        error = angle-yawAngle(checkNegative)
        if abs(error) < abs(halfYaw):
            speed = speed1+(speedDiff1*((yawAngle(checkNegative) - yawStart)/(halfYaw)))
        else:
            speed = speed3+(speedDiff2*((angle - yawAngle(checkNegative))/(halfYaw)))
    if speed2 * angle > 0:
        motor_pair.move_tank(motor_pair.PAIR_1, speed, 0 - speed * motorSplit)
    else:
        motor_pair.move_tank(motor_pair.PAIR_1,0 - speed * motorSplit, speed1)
'''

def yawTurn(speed1, speed2, speed3, angle, motorSplit, checkNegative):
    """
    Turn the robot to a target yaw angle using gradual acceleration and deceleration.

    speed1: initial speed
    speed2: peak speed
    speed3: final speed
    angle: target yaw angle
    motorSplit: differential speed ratio for turning
    checkNegative: whether to allow negative yaw readings
    tolerance: stopping tolerance in degrees
    """

    yawStart = yawAngle(checkNegative)
    totalYaw = angle - yawStart

    if totalYaw == 0:
        return

    speedDiff1 = speed2 - speed1# accel
    speedDiff2 = speed2 - speed3# decel

    halfYaw = totalYaw / 2

    while True:
        currentYaw = yawAngle(checkNegative)
        error = angle - currentYaw

        if abs(error) <= 10:
            break

        # Determine speed based on distance to target
        if abs(error) > abs(halfYaw):
            # Far from target → speed up
            speed = speed1 + speedDiff1 * (abs(error) / abs(totalYaw))
        else:
            # Near target → slow down
            speed = speed3 + speedDiff2 * (abs(error) / abs(halfYaw))

        # Clamp speed to safe range
        speed = max(min(speed, max(speed1, speed2, speed3)), 0)

        # Determine motor directions
        if error > 0:
            leftSpeed = speed
            rightSpeed = speed * motorSplit
        else:
            leftSpeed = speed * motorSplit
            rightSpeed = speed

        motor_pair.move_tank(motor_pair.PAIR_1, leftSpeed, rightSpeed)

async def main():
    # write your code here
    motion_sensor.reset_yaw(0)## reset yaw
    gyroFwd(600, 1000, 0, 0) ##Start
    gyroFwd(500, 1000, 900, 0)
    gyroFwd(500,1000,1800, 1)
    gyroFwd(500,1000, -900, -1)
    gyroFwd(500, 1000, 0, 0)
    motor_pair.stop(motor_pair.PAIR_1)##stop

    ##yawTurn(1000,1000,1000, -900, 1, 0) ##Turn to grab RSMP


    
    gyroRmpFwd(300,500,1000,0,0)
    gyroFwd(600,1000,0,0)
    gyroRmpFwd(200,1000,500,0,0)
    yawTurn(300,1000,300,-900,1,0)


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

