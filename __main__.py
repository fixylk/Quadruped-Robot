from MoveForward import MoveForward
from Rotate import *
from SlowStepForward import *
from LegsControl import *

rear_left = RobotLeg(6, 5, m2_revert=2, servoMinOffset=-0.05)
rear_right = RobotLeg(10, 9, m2_revert=1, m1_revert=3, servoMaxOffset=-0.05, servoMinOffset=-0.25)
front_left = RobotLeg(3, 2, m2_revert=1, m1_revert=3, servoMinOffset=-0.15)
front_right = RobotLeg(11, 12, m2_revert=2, servoMinOffset=-0.10)

move_forward = MoveForward(front_left, front_right, rear_left, rear_right)

if __name__ == '__main__':

    KeepAliveThread()

    rear_left.move_h1(MIN_h1_rear)
    rear_right.move_h1(MIN_h1_rear)
    front_left.move_h1(MAX_h1_front)
    front_right.move_h1(MAX_h1_front)

    for x in range(-90, MAX_h2_front):
        rear_left.move_h2(x*MAX_h2_rear/MAX_h2_front)
        rear_right.move_h2(x*MAX_h2_rear/MAX_h2_front)
        front_left.move_h2(x)
        front_right.move_h2(x)
        time.sleep(1.5 * 0.005)

    while True:
        desiredPosition1 = input("value1? ")
        # desiredPosition2 = input("value2? ")
        SlowStepForward(front_left, front_right, rear_left, rear_right,Factor=0.4)
        # move_forward.move()
        # time.sleep(3)
        # move_forward.stop()
        #
        # RotateRight(front_left, front_right, rear_left, rear_right)
        # RotateRight(front_left, front_right, rear_left, rear_right)
        #
        # move_forward.move()
        # time.sleep(3)
        # move_forward.stop()
        #
        # RotateLeft(front_left, front_right, rear_left, rear_right)
        # RotateLeft(front_left, front_right, rear_left, rear_right)

        # rear_left.move_h1(desiredPosition1)
        # rear_right.move_h1(desiredPosition1)
        #
        # rear_left.move_h2(desiredPosition2)
        # rear_right.move_h2(desiredPosition2)
        #
        # front_left.move_h1(desiredPosition1)
        # front_right.move_h1(desiredPosition1)
        #
        # front_left.move_h2(desiredPosition2)
        # front_right.move_h2(desiredPosition2)
        pass
