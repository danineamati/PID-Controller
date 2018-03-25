import numpy as np # mathematics
import matplotlib.pyplot as plt # Plotting

import serial # Communication to Arduino

class PID:

    def __init__(self):
        self.P = 1
        self.I = 1
        self.D = 1

    def GetParameters(self):
        return [self.P, self.I, self.D]

    def printParameters(self):
        param = zip(['Proportional Term: ', \
                     'Integral Term:     ', \
                     'Differential Term: '], self.GetParameters())

        for var in param:
            print(var[0], var[1])

    def setP(self, PNew):
        self.P = PNew


def main():
    ''' This is the main function that creates an instance of the
    PID controller and tests it.
    This function does not return anything.'''

    print("PID Controller")
    control = PID()
    control.printParameters() 
    

if __name__ == "__main__":
    main()
