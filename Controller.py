"""
File: Controller.py
Author: Roushan Nigam Shaw
Date: 26-04-2024
Correspondance: roshannigams6@gmail.com

Description: Implementation of P, PI & PID controllers in time domain in python.
To be implemented: Input Handling, Anti-windup for Integrator, 
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from ipywidgets import interact, fixed

class controllers:
    # Class attribute
    dt = 0.01
    tauI = 1
    tauD = 1

    def __init__(self, totalTime, kp, ki, kd, inpType):
        self.totalTime = totalTime
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.inpType = inpType
    
    # pController method
    def P_Controller(self, sp, bias, stepAfterTime):

        #time discretization
        dt = self.dt
        timeDiscreetization = np.arange(0, self.totalTime, dt)
        step = np.zeros_like(timeDiscreetization)

        #input type checker
        if self.inpType == "step":
            step[timeDiscreetization >= stepAfterTime] = sp
        else:
            raise TypeError("Only step/ramp/pulse inputs are allowed.")

        #pcontroller
        Pt = np.zeros(len(step))
        Pt[0] = bias
        error = sp - Pt[0]
        er = [0]
        for i in range(len(step)):
            if timeDiscreetization[i] >= stepAfterTime:
                Pt[i] = Pt[i-1] + ((bias + self.kp * error)) * dt #euler method
                disturbance = np.random.normal(0.0005, 0.001) #introducing some disturbance
                Pt[i] += disturbance
                error = sp - Pt[i]
                
        #figure plotting
        plt.figure()
        plt.title("P-controller")
        plt.plot(timeDiscreetization, step, label="step_input")
        plt.plot(timeDiscreetization, Pt, label="CO")
        plt.grid(True)
        plt.show()
    
    def PI_Controller(self, sp, bias, stepAfterTime, tauI):

        #time discretization
        dt = self.dt
        timeDiscreetization = np.arange(0, self.totalTime, dt)
        step = np.zeros_like(timeDiscreetization)

        #input type checker
        if self.inpType == "step":
            step[timeDiscreetization >= stepAfterTime] = sp
        else:
            raise TypeError("Only step/ramp/pulse inputs are allowed.")
        
        #system specific
        Ki = self.ki
        #-----#
        #PI controller
        Pt = np.zeros(len(step))
        Pt[0] = bias
        error = sp - Pt[0]
        erdtSum = 0
        for i in range(len(step)):
            if timeDiscreetization[i] >= stepAfterTime:
                Pt[i] = Pt[i-1] + (((bias + self.kp * error)) * dt) +  ((Ki/tauI)*(erdtSum)) #euler method
                disturbance = np.random.normal(0.0005, 0.001) #introducing some disturbance
                Pt[i] += disturbance
                error = sp - Pt[i]
                erdtSum += error*dt

        #Figure Plotting
        plt.figure()
        plt.title("PI-controller")
        plt.plot(timeDiscreetization, step, label="step_input")
        plt.plot(timeDiscreetization, Pt, label="CO")
        plt.grid(True)
        plt.show()

    def PID_Controller(self, sp, bias, stepAfterTime, tauI, tauD):

        #time discretization
        dt = self.dt
        timeDiscreetization = np.arange(0, self.totalTime, dt)
        step = np.zeros_like(timeDiscreetization)

        #input type checker
        if self.inpType == "step":
            step[timeDiscreetization >= stepAfterTime] = sp
        else:
            raise TypeError("Only step/ramp/pulse inputs are allowed.")
        
        #system specific
        Ki = self.ki
        Kd = self.kd
        #-----#

        #PID controller
        Pt = np.zeros(len(step))
        Pt[0] = bias
        error = sp - Pt[0]
        erdtSum = 0
        er = np.zeros(len(step))
        for i in range(len(step)):
            if timeDiscreetization[i] >= stepAfterTime:
                Pt[i] = Pt[i-1] + (((bias + self.kp * error)) * dt) +  ((Ki/tauI)*(erdtSum)) + ((Kd*tauD)*(er[i]-er[i-1]))#euler method
                disturbance = np.random.normal(0.0005, 0.001) #introducing some disturbance
                Pt[i] += disturbance
                error = sp - Pt[i]
                erdtSum += error*dt
                er[i] = error

        #Figure plotting
        plt.figure()
        plt.title("PID-controller")
        plt.plot(timeDiscreetization, step, label="step_input")
        plt.plot(timeDiscreetization, Pt, label="CO")
        plt.grid(True)
        plt.show()

# Parameters
sp = 1
kp = 20
ki = 1
kd = 0.1
bias = 0
totalTime = 8
stepAfterTime = 2 #Make it in a way that it accepts an array

# Create an instance of controllers class
controller = controllers(totalTime, kp, ki, kd, inpType="step")

# Call the pController method
controller.P_Controller(sp, bias, stepAfterTime)
controller.PI_Controller(sp, bias, stepAfterTime, tauI = 1)
controller.PID_Controller(sp, bias, stepAfterTime, tauI = 1, tauD = 1)
