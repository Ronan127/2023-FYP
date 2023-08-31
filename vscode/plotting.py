import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
import random
import serial

#initialize serial port
ser = serial.Serial()
ser.port = 'COM4' #Arduino serial port
ser.baudrate = 115200
ser.timeout = 10 #specify timeout when using readline()
ser.parity = serial.PARITY_EVEN
ser.open()
if ser.is_open==True:
	print("\nAll right, serial port now open. Configuration:\n")
	print(ser, "\n") #print serial parameters

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = [] #store trials here (n)
ys = [] #store relative frequency here
rs = [] #for theoretical probability
xstart=0
# Format plot
plt.xticks(rotation=45, ha='right')
plt.subplots_adjust(bottom=0.30)
plt.title('Current over time')
plt.ylabel('current')
plt.legend()
plt.axis([None, None, None, None]) #Use for arbitrary number of trials
#plt.axis([1, 100, 0, 1.1]) #Use for 100 trial demo

# This function is called periodically from FuncAnimation
def animate(i, xs, ys):
    global xstart
    for i in range(0, 200):
        #Aquire and parse data from serial port
        line=ser.readline()      #ascii
        print(line)
        line_as_list = line.split(b',')
        i = int(line_as_list[0])
        relCurrent = line_as_list[1]
        relCurrent_as_list = relCurrent.split(b'\n')
        relCurrent_float = abs(float(relCurrent_as_list[0]))
    
    # Add x and y to lists
    if len(xs) == 0:
        xstart=i
    if i==1:
            xs = []
            ys= []
            xstart = 0
            ax.clear()
    xs.append((i-xstart)/200)
    ys.append(relCurrent_float)

    # Limit x and y lists to 20 items
    #xs = xs[-20:]
    #ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys, label="Current")



# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1)
plt.show()