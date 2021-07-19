from LegsControl import *
import time


def SlowStepForward(front_left, front_right, rear_left, rear_right, Factor = 1.0):
    front_left_r = Process(target=FrontLeg, args=(front_left,Factor,True,))
    rear_right_r = Process(target=RearLeg,args=(rear_right,Factor,True,))
    front_left_r.start()
    rear_right_r.start()
    time.sleep(MAX_h1_front*Factor * DELAY)
    front_right_r = Process(target=FrontLeg, args=(front_right,Factor,))
    rear_left_r = Process(target=RearLeg, args=(rear_left,Factor,))
    front_right_r.start()
    rear_left_r.start()
    front_left_r.join()
    rear_right_r.join()
    front_right_r.join()
    rear_left_r.join()


def FrontLeg(leg,Factor = 1.0, afterUP = False):
    leg.move_h2(MIN_h2)

    for x in range(MAX_h1_front, int(MAX_h1_front*(1-Factor)), -1):
        leg.move_h1(x)
        time.sleep(DELAY)

    leg.move_h2(MAX_h2_front)
    for x in range(int(MAX_h1_front*(1-Factor)), MAX_h1_front):
        leg.move_h1(x)
        time.sleep(DELAY)

    if afterUP:
        leg.move_h2(MIN_h2)
        time.sleep(MAX_h1_front * Factor * DELAY)
        leg.move_h2(MAX_h2_front)


def RearLeg(leg, Factor = 1.0, afterUP = False):
    leg.move_h2(MIN_h2)

    for x in range(MIN_h1_rear, int(MIN_h1_rear*(1+Factor))):
        leg.move_h1(x)
        time.sleep(DELAY)

    leg.move_h2(MAX_h2_rear)
    for x in range(int(MIN_h1_rear*(1+Factor)), MIN_h1_rear, -1):
        leg.move_h1(x)
        time.sleep(DELAY)

    if afterUP:
        leg.move_h2(MIN_h2)
        time.sleep(MAX_h1_front * Factor * DELAY)
        leg.move_h2(MAX_h2_rear)

