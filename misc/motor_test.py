import sabertooth2x12 as SB
import time

motors=SB.Sabertooth()

print("one")
motors.driveMotor('left', 'rev', 50)

time.sleep(2)
print("two")
motors.driveMotor('right', 'fwd', 50)

time.sleep(2)
print("three")
motors.driveMotor('both', 'fwd', 10)

time.sleep(2)
print("four")
motors.driveMotor('left', 'rev', 10)

time.sleep(2)
print("five")
motors.driveMotor('right', 'rev', 10)

time.sleep(2)
print("six")
motors.driveMotor('both', 'fwd', 50)

time.sleep(2)
print("seven")
motors.driveMotor('both', 'fwd', 0)

print("one")
motors.driveMotor('left', 'rev', 50)

time.sleep(2)
print("two")
motors.driveMotor('right', 'fwd', 50)

time.sleep(2)
print("three")
motors.driveMotor('both', 'fwd', 10)

time.sleep(2)
print("four")
motors.driveMotor('left', 'rev', 10)

time.sleep(2)
print("five")
motors.driveMotor('right', 'rev', 10)

time.sleep(2)
print("six")
motors.driveMotor('both', 'fwd', 50)

time.sleep(2)
print("seven")
motors.driveMotor('both', 'fwd', 0)
