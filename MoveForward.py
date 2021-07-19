from MoveLegRear import *
from LegsControl import *
from MoveLegFront import MoveLegFront


class MoveForward:
    def __init__(self, front_left, front_right, rear_left, rear_right):
        self.front_left = front_left
        self.rear_right = rear_right
        self.front_right = front_right
        self.rear_left = rear_left
        self.move_front_left = MoveLegFront(front_left)
        self.move_rear_right = MoveLegRear(rear_right)
        self.move_front_right = MoveLegFront(front_right)
        self.move_rear_left = MoveLegRear(rear_left)
        self.isMoving = False

    def move(self):
        self.move_front_left.start()
        self.move_rear_right.start()
        time.sleep(h1_moving_angle * DELAY)
        self.move_front_right.start()
        self.move_rear_left.start()
        self.isMoving = True

    def stop(self):
        self.move_front_left.shutdown()
        self.move_rear_right.shutdown()
        self.move_front_right.shutdown()
        self.move_rear_left.shutdown()
        self.move_front_left.join()
        self.move_rear_right.join()
        self.move_front_right.join()
        self.move_rear_left.join()
        self.isMoving = False
        self = self.__init__(self.front_left, self.front_right, self.rear_left, self.rear_right)
