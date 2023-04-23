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
timeBurnout = -1
for line in outLines:
    data = line.split()
    # print(data)
    if len(data) < 13:
       continue
    realTimes.append(float(data[0]))
    realPosition.append([float(data[1]),float(data[2]),float(data[3])])
    realZAxis.append([float(data[10]),float(data[11]),float(data[12])])
    if timeBurnout < 0 and float(data[13]) < 0.1:
        timeBurnout = float(data[0])

print('Time Burnout')
print(timeBurnout)

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
pitchReal = []
pitchFiltered = []
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

    pitchReal.append(np.arccos(realZAxis[i][2]))
    pitchFiltered.append(np.arccos(filteredZAxis[i][2]))

iLast = len(realTimes);
realAcceleration = np.zeros(iLast)
i = 2
dt = 1.0/(9.806*(realTimes[1] - realTimes[0]))
pos_prev = np.array(realPosition[0])
pos_next = np.array(realPosition[1])
while (i < iLast):
    pos = pos_next
    pos_next = np.array(realPosition[i])

    acc = (pos_next - 2*pos + pos_prev)

    g = np.linalg.norm(acc)*dt;

    realAcceleration[i-1] = g

    pos_prev = pos;
    i += 1

# fig = plt.figure()
# ax = plt.axes()
# ax.plot(realTimes,realAcceleration,'b')

fig = plt.figure()
ax = plt.axes()

ax.plot(realTimes,err,'r')
ax.plot(realTimes,realHeight,'g')
ax.plot(realTimes,filteredHeight,':b')
ax.legend(['err','real','filtered'])
ax.set_xlim([0,50])
ax.set_ylim([-5,maxHeight + 20])

fig = plt.figure()
ax = plt.axes()

ax.plot(realTimes,angleErr,'r')
ax.plot(realTimes,pitchReal,'g');
ax.plot(realTimes,pitchFiltered,':b')
ax.legend(['err','real','filtered'])

fig = plt.figure()
ax = plt.axes(projection ='3d')

ax.plot3D(realX,realY,realZ, 'r')
ax.plot3D(filteredX,filteredY,filteredZ, ':b')
ax.set_zlim([-5,maxHeight + 20])
ax.set_xlim([-10 - maxHeight*0.5,maxHeight*0.5 + 10])
ax.set_ylim([-10 - maxHeight*0.5,maxHeight*0.5 + 10])

ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

plt.show()

