from hub import light_matrix
import motion_sensor
import runloop
import port
import motor
import motor_pair


motor_pair.pair(motor_pair.PAIR_1, port.A, port.E)

kp = 2
checkNegative = 0


##FUNCTIONS

##Yaw Adjust
def yawAngle(negative)
    yaw = motion_sensor.tilt_angles()[0]
    if negative == 1 and yaw < 0:
        return yaw + 360
    elif negative == -1 and yaw > 0:
        return yaw - 360
    else:
        return yaw

##GyroFwd

def gyroFwd(degrees, speed, targetAngle):
    motor.reset_relative_position(port.E, 0) ##Reset relative position

    while abs(motor.relative_position(port.A))<degrees: ## Wait until traveled specified degrees
        currentAngle = yawAngle(checkNegative) ## Get yaw
        error = currentAngle - targetAngle ## Find error
        proportional = error*kp ## Calculate Proportional
        correction = proportional ## Calculate Correction
        motor_pair.move_tank(motor_pair.PAIR_1, speed+correction, speed-correction) ##Set speed

##Yaw Turn

def yawTurn(speed1, speed2, speed3, angle, motorSplit):
    yawStart = yawAngle(checkNegative)
    speed = speed1
    speedDiff1 = speed1 - speed2
    speedDiff2 = speed2 - speed3
    error = angle-yawAngle(checkNegative)
    while not abs(error) < 1:
        error = angle-yawAngle(checkNegative):
        if abs(error) < 1:
            speed = speed1+(speedDiff1*((yawAngle(checkNegative) - yawStart)/(angle/2)))
        else:
            speed = speed3+(speedDiff2*((angle - yawAngle(checkNegative))/(angle/2)))
    if speed2 * angle > 0:
        motor_pair.move_tank(motor_pair.PAIR_1, speed, 0 - speed * motorSplit)
    else:
        motor_pair.move_tank(motor_pair.PAIR_1,  0 - speed * motorSplit, speed)

async def main():
    # write your code here
    checkNegative = 0
    gyroFwd(1300, 1000, 0) ##Start
    yawTurn(1000,1000,1000, -90, 1) ##Turn to grab RSMP
    gyroFwd(200, 500, -900) ##Grab RSMP
    gyroFwd(200, 1000, -900)
    gyroFwd(200, 500, -900)
    checkNegative = -1
    gyroFwd(1000, 1000, -1650) ##Send RSMP
    gyroFwd(200, 1000, -1800)
    yawTurn(1000,1000,1000, 0, 1)
    checkNegative = 0
    gyroFwd(1500, 1000, 0) ##Send Drone
    motor_pair.stop(motor_pair.PAIR_1)

runloop.run(main())


