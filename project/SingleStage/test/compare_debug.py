import numpy as np
import matplotlib.pyplot as plt

def dot(u,v):
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]

outFile = open('out.txt','r')
outLines = outFile.readlines()
outFile.close()
outLines.pop(0)

debugFile = open('debug.txt','r')
debugLines = debugFile.readlines()
debugFile.close()

realTimes = []
realPosition = []
realZAxis = []
for line in outLines:
    data = line.split()
    # print(data)
    if len(data) < 13:
       continue
    realTimes.append(float(data[0]))
    realPosition.append([float(data[1]),float(data[2]),float(data[3])])
    realZAxis.append([float(data[10]),float(data[11]),float(data[12])])

filterTimes = []
filterPosition = []
filteredZAxis = []
for line in debugLines:
    data = line.split()
    if len(data) < 13:
       continue
    filterTimes.append(float(data[0]))
    filterPosition.append([float(data[1]),float(data[2]),float(data[3])])
    filteredZAxis.append([float(data[13]),float(data[14]),float(data[15])])

err = []
angleErr = []
realHeight = []
filteredHeight = []
maxHeight = 0
realX = []
realY = []
realZ = []
filteredX = []
filteredY = []
filteredZ = []
for i in range(len(realTimes)):
    real = np.array(realPosition[i])
    filtered = np.array(filterPosition[i])
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

    proj = dot(realZAxis[i],filteredZAxis[i])
    angleErr.append(np.arccos(proj))


fig = plt.figure()
ax = plt.axes()

ax.plot(filterTimes,err)
ax.plot(filterTimes,realHeight)
ax.plot(filterTimes,filteredHeight,':g')
ax.legend(['err','real','filtered'])
ax.set_xlim([0,50])
ax.set_ylim([-5,maxHeight + 20])

fig = plt.figure()
ax = plt.axes()

ax.plot(filterTimes,angleErr)

fig = plt.figure()
ax = plt.axes(projection ='3d')

ax.plot3D(realX,realY,realZ, 'r')
ax.plot3D(filteredX,filteredY,filteredZ, ':g')

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

plt.show()

