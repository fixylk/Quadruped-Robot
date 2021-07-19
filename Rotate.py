from LegsControl import *


def RotateRight(front_left, front_right, rear_left, rear_right):
    front_left_r = Process(target=frontLeg,args=(front_left,))
    rear_right_r = Process(target=rearLeg,args=(rear_right,))
    upLegs = Process(target=legsUP,args=(front_right,rear_left,))
    front_left_r.start()
    rear_right_r.start()
    upLegs.start()
    front_left_r.join()
    rear_right_r.join()
    upLegs.join()

def RotateLeft(front_left, front_right, rear_left, rear_right):
    front_left_r = Process(target=frontLeg,args=(front_right,))
    rear_right_r = Process(target=rearLeg,args=(rear_left,))
    upLegs = Process(target=legsUP,args=(front_left,rear_right,))
    front_left_r.start()
    rear_right_r.start()
    upLegs.start()
    front_left_r.join()
    rear_right_r.join()
    upLegs.join()

def frontLeg(leg):
    leg.move_h2(MIN_h2)

    for x in range(MAX_h1_front, MIN_h1_front, -1):
        leg.move_h1(x)
        time.sleep(DELAY)

    leg.move_h2(MAX_h2_front)
    for x in range(MIN_h1_front, MAX_h1_front):
        leg.move_h1(x)
        time.sleep(DELAY)

def rearLeg(leg):
    leg.move_h2(MIN_h2)

    for x in range(MIN_h1_rear, MIN_h1_rear-h1_moving_angle,-1):
        leg.move_h1(x)
        time.sleep(DELAY)

    leg.move_h2(MAX_h2_rear)
    for x in range(MIN_h1_rear-h1_moving_angle, MIN_h1_rear):
        leg.move_h1(x)
        time.sleep(DELAY)

def legsUP(legFront, legRear):
    time.sleep((h1_moving_angle+10) * DELAY)
    legFront.move_h2(MIN_h2)
    legRear.move_h2(MIN_h2)
    time.sleep((h1_moving_angle-10) * DELAY)
    legFront.move_h2(MAX_h2_front)
    legRear.move_h2(MAX_h2_rear)
