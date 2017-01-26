import serial
import matplotlib.pyplot as plt
from matplotlib import style
import numpy as np
import math

style.use('fivethirtyeight')
# initalize interactive matplotlib
plt.ion()

ydata = [0] * 50
ax1=plt.axes()
line, = plt.plot(ydata)
plt.ylim([0,650])

ser = serial.Serial('com11', 9600)

print(ydata)
while True:
    data = ((str(ser.readline()).replace("\'", "")).replace("\\r\\n","")).split(" Distance: ")
    if len(data) == 2:
        ymin = float(min(ydata))-10
        ymax = float(max(ydata))+10
        plt.ylim([ymin,ymax])
        volts=float(data[1])*.0048828125
        print(type(volts))
        dist = -(3.078*math.pow(volts,5))+(29.645*math.pow(volts,4))-(110.68*math.pow(volts,3))+(201.94*math.pow(volts,2))-(186.84*math.pow(volts,1))+81.524
        ydata.append(dist)
        del ydata[0]
        line.set_xdata(np.arange(len(ydata)))
        line.set_ydata(ydata)  # update the data
        plt.draw()
        plt.pause(0.1)