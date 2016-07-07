import sabertooth2x12 as SB
import time

motors=SB.Sabertooth()
print("one")
motors.driveMotor('left', 'rev', 10)


time.sleep(2)
print("two")
motors.driveMotor('right', 'fwd', 10)

time.sleep(2)
print("three")
motors.driveMotor('both', 'fwd', 10)
