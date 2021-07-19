from LegsControl import *
import time
import multiprocessing


class MoveLegFront(multiprocessing.Process):

    def __init__(self, leg):
        super(MoveLegFront, self).__init__()
        self.leg = leg
        self.exit = multiprocessing.Event()

    def run(self):
        while not self.exit.is_set():
            self.leg.move_h2(MIN_h2)

            for x in range(MAX_h1_front, MIN_h1_front, -1):
                self.leg.move_h1(x)
                time.sleep(DELAY)

            self.leg.move_h2(MAX_h2_front)
            for x in range(MIN_h1_front, MAX_h1_front):
                self.leg.move_h1(x)
                time.sleep(DELAY)

    def shutdown(self):
        self.exit.set()
