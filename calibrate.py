import serial
import matplotlib.pyplot as plt
from matplotlib import style
import numpy as np
import math
import csv

style.use('fivethirtyeight')
# initalize interactive matplotlib
plt.ion()

ydata = [0] * 50
ax1=plt.axes()
line, = plt.plot(ydata)
plt.ylim([0,650])

ser = serial.Serial('com11', 9600)

print(ydata)
with open('results.csv', 'a') as csvfile:
    while True:
        data = ((str(ser.readline()).replace("\'", "")).replace("\\r\\n","")).split(" Distance: ")
        if len(data) == 2:
            ymin = float(min(ydata))-10
            ymax = float(max(ydata))+10
            plt.ylim([ymin,ymax])
            volts=float(data[1])*.0048828125
            print(type(volts))
            datawriter = csv.writer(csvfile, delimiter=',',
                                    quotechar='\'', quoting=csv.QUOTE_MINIMAL)
            datawriter.writerow([volts])
            ydata.append(volts)
            del ydata[0]
            line.set_xdata(np.arange(len(ydata)))
            line.set_ydata(ydata)  # update the data
            plt.draw()
            plt.pause(0.1)