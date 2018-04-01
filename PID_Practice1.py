import numpy as np # mathematics
import matplotlib.pyplot as plt # Plotting
import time
import textwrap as tw

import serial # Communication to Arduino

class PID:
    '''This is a class for a PID (Proportional, Integral, Derivative)
    controller.
    u(t) = P * error(t) + I * /int error(t') dt' - D derror(t)/dt
    '''

    def __init__(self, P = 1, I = 1, D = 1, val = 0):
        self.P = P
        self.I = I
        self.D = D
        self.setpt = val
        self.start_time = time.time()
        self.data = []
        self.allData = []
        self.lastData = val

    def GetParameters(self):
        '''Returns P, I, D parameters in this order.'''
        return [self.P, self.I, self.D, self.setpt]

    def formatParameters(self):
        '''Formats parameters to print in plot'''
        param = zip(['Proportional Term: ', \
                     'Integral Term:     ', \
                     'Differential Term: ', \
                     'Set Point:         '], self.GetParameters())
        paramStr = ''
        for var in param:
            paramStr += var[0] + ' ' + str(var[1]) + '\n'
            
        return paramStr

    def printParameters(self):
        '''Formats and prints parameters.'''
        param = zip(['Proportional Term: ', \
                     'Integral Term:     ', \
                     'Differential Term: ', \
                     'Set Point:         '], self.GetParameters())

        for var in param:
            print(var[0], var[1])
    
    def getP(self):
        '''This returns the Proportional Term.'''
        return self.P

    def getI(self):
        '''This returns the Integral Term.'''
        return self.I

    def getD(self):
        '''This returns the Differential Term.'''
        return self.D

    def getSetPt(self):
        '''Returns the controller setpoint.'''
        return self.setpt

    def setP(self, PNew):
        '''This sets a new Proportional Term.'''
        self.P = PNew

    def setI(self, INew):
        '''This sets a new Integral Term.'''
        self.I = INew

    def setD(self, DNew):
        '''This sets a new Differential Term.'''
        self.D = DNew

    def uploadData(self, dataNew):
        '''This sets the data list to a new set of data.'''
        self.data = dataNew

    def addDataPoint(self, dataPoint):
        '''Appends data point to data list.'''
        self.data.append(dataPoint)
        self.allData.append(dataPoint)
        self.lastData = dataPoint

    def updateSetPoint(self, setptNew):
        '''Updates set point, clears data, and resets time.'''
        self.setpt = setptNew
        current = self.lastData
        self.data = [current]
        self.resetTime()

    def error(self, value):
        '''This returns the difference between the setpoint and the value
        in consideration.'''
        return self.setpt - value

    def resetTime(self):
        '''Resets the time to the current time.'''
        self.start_time = time.time()

    def elapsedTime(self):
        return time.time() - self.start_time

    def KpTerm(self):
        '''This is the impact of the proportional term on the output.'''
        return self.P * self.error(self.data[len(self.data) - 1])

    def KiTerm(self):
        '''This is the impact of the integration term on the output.'''
        # First we need to calculate the total error
        total_error = 0
        for dataPt in self.data:
            total_error += self.error(dataPt)
        
        return self.I * (total_error)

    def KdTerm(self):
        '''This is the impact of the differential term on the output.'''
        if (len(self.data) < 2): # Edge Case
            prev_data = self.data[len(self.data) - 1]
        else:
            prev_data = self.data[len(self.data) - 2]

        prev_error = self.error(prev_data)
        current_error = self.error(self.data[len(self.data) - 1])
        return self.D * (current_error - prev_error)

    def output(self):
        '''This is the main PID function. Outputting how the controller
        needs to change.'''
        return self.KpTerm() + self.KiTerm() + self.KdTerm()

def makePlot(controller, setPointCurve, active = False, legend = False):
    '''Makes plot for given controller.'''

    if not active:
        handle = plt.figure(1)
        plt.plot(controller.allData)
        plt.title("Measurement against time.")
        plt.xlabel("Time (ticks)")
        plt.ylabel("Measurement")

        plt.plot(setPointCurve[0], setPointCurve[1])

        plt.legend([controller.formatParameters(), "Set Point"], loc = 'best')

        plt.show()

    else:
        plt.ion()
        handle = plt.figure(1,figsize=(8, 6))
        plt.plot(controller.allData)
        plt.title("Measurement against time.")
        plt.xlabel("Time (ticks)")
        plt.ylabel("Measurement")

        plt.plot(setPointCurve)

        if legend:
            plt.legend([controller.formatParameters(), "Set Point"], \
                       loc = 'best') 

        handle.tight_layout()

        plt.pause(0.1)

    return handle

def activePrint(controller):
    print()
    print("Proportional:", controller.KpTerm())
    print("Integral:    ", controller.KiTerm())
    print("Differential:", controller.KdTerm())
    print("---------------------")
    print("Output:      ", controller.output())

def loop(controller):
    for i in range(25):
        controller.addDataPoint(controller.lastData + \
                                controller.output())

def activeloop(controller):
    setPointCurve = []
    Legend = True
    start_time = time.time()
    input("Ready? ")
    
    while time.time() - start_time < 100000:
        controller.addDataPoint(controller.lastData + \
                                controller.output())
        setPointCurve.append(controller.getSetPt())
        handle = makePlot(controller, setPointCurve, \
                          active = True, legend = Legend)
        Legend = False

        if time.time() - start_time > 1:
            newSet = input("New Setpoint: ")
            start_time = time.time()
            if newSet == 'q':
                plt.close(handle)
                break
            elif newSet == 'h':
                plt.pause(10)
            try:
                controller.updateSetPoint(float(newSet))
            except ValueError:
                pass

def main():
    ''' This is the main function that creates an instance of the
    PID controller and tests it.
    This function does not return anything.'''

##    print()
##    print("PID Controller 1")
##    control = PID(0.5, 0.2, .3,500)
##    control.printParameters()
##    control.addDataPoint(800)
##    loop(control)
##
##    makePlot(control, ([0, 50], [500, 500]))

##    current = control.lastData
##
##    control.updateSetPoint(600)
##    loop(control)
##
##    makePlot(control, ([0, 25, 25, 50], [500, 500, 600, 600]))

    print()
    print("PID Controller 2")
    control2 = PID(0.5, 0.2, .3,500)
    control2.printParameters()
    control2.addDataPoint(800)
    activeloop(control2)
    

if __name__ == "__main__":
    main()
