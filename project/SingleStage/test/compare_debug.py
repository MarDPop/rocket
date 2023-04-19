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
maxHeight = 0
realX = []
realY = []
realZ = []
filteredX = []
filteredY = []
filteredZ = []
for i in range(len(filterTimes)):
    real = np.array(realStates[i])
    filtered = np.array(filterStates[i])
    diff = real - filtered
    err.append(np.linalg.norm(diff))
    realHeight.append(real[2])
    filteredHeight.append(filtered[2])

    if real[2] > maxHeight:
        maxHeight = real[2]

    realX.append(real[0])
    realY.append(real[1])
    realZ.append(real[2])

    filteredX.append(filtered[0])
    filteredY.append(filtered[1])
    filteredZ.append(filtered[2])
    

plt.plot(filterTimes,err)
plt.plot(filterTimes,realHeight)
plt.plot(filterTimes,filteredHeight,':g')
plt.legend(['err','real','filtered'])
plt.xlim([0,50])
plt.ylim([-5,maxHeight + 20])

fig = plt.figure()
ax = plt.axes(projection ='3d')

ax.plot3D(realX,realY,realZ, 'r')
ax.plot3D(filteredX,filteredY,filteredZ, ':g')

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

plt.show()

