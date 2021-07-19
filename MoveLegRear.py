from LegsControl import *
import time
import multiprocessing


class MoveLegRear(multiprocessing.Process):

    def __init__(self, leg):
        super(MoveLegRear, self).__init__()
        self.leg = leg
        self.exit = multiprocessing.Event()

    def run(self):
        while not self.exit.is_set():
            self.leg.move_h2(MIN_h2)

            for x in range(MIN_h1_rear, MAX_h1_rear):
                self.leg.move_h1(x)
                time.sleep(DELAY)

            self.leg.move_h2(MAX_h2_rear)
            for x in range(MAX_h1_rear, MIN_h1_rear, -1):
                self.leg.move_h1(x)
                time.sleep(DELAY)

    def shutdown(self):
        self.exit.set()
