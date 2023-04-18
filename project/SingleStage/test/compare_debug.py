import numpy as np
import matplotlib.pyplot as plt

outFile = open('out.txt','r')
outLines = outFile.readlines()
outFile.close()

debugFile = open('debug.txt','r')
debugLines = debugFile.readlines()
debugFile.close()

realTimes = []
realStates = []
for line in outLines:
    data = line.split()
    if len(data) < 5:
       continue
    realTimes.append(float(data[0]))
    realStates.append([float(data[1]),float(data[2]),float(data[3])])

filterTimes = []
filterStates = []
for line in debugLines:
    data = line.split()
    if len(data) < 5:
       continue
    filterTimes.append(float(data[0]))
    filterStates.append([float(data[1]),float(data[2]),float(data[3])])

err = []
realHeight = []
filteredHeight = []
for i in range(len(filterTimes)):
    real = np.array(realStates[i])
    filtered = np.array(filterStates[i])
    diff = real - filtered
    err.append(np.linalg.norm(diff))
    realHeight.append(real[2])
    filteredHeight.append(filtered[2])

plt.plot(filterTimes,err)
plt.plot(filterTimes,realHeight)
plt.plot(filterTimes,filteredHeight)
plt.legend(['err','real','filtered'])
plt.show()
    
