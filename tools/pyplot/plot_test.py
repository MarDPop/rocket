import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import sys
import tkinter as tk
import time

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg,
    NavigationToolbar2Tk
)

class App(tk.Tk):

    def __init__(self):
        super().__init__()
        self.nData = 1
        self.times = np.zeros(1)
        self.position = np.zeros((1,3))
        self.velocity = np.zeros((1,3))
        self.acceleration = np.zeros((1,3))
        self.xAxis = np.zeros((1,3))
        self.yAxis = np.zeros((1,3))
        self.zAxis = np.zeros((1,3))
        self.t_burnout = 0

        self.figure = Figure(figsize=(6, 4), dpi=100)
        figure_canvas = FigureCanvasTkAgg(self.figure, self)
        NavigationToolbar2Tk(figure_canvas, self)
        self.axes = self.figure.add_subplot(111,projection="3d")
        figure_canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        #figure_canvas.get_tk_widget().grid(row=2,column=1)

        self.dataframe = tk.Frame(self)
        self.dataframe.pack()    

        label = tk.Label(self.dataframe,text="file location")
        label.grid(row=1,column=1)

        self.filename = tk.Entry(self.dataframe)
        self.filename.grid(row=1,column=2)

        btn = tk.Button(self.dataframe,text="load", command=self.load)
        btn.grid(row=1,column=3)

        btn = tk.Button(self.dataframe,text="animate", command=self.animate)
        btn.grid(row=1,column=4)

    def load(self):
        fn = self.filename.get()
        if fn == '':
            fn = "../../test/out.txt"
        
        lines = []
        with open(fn,'r') as data:
            lines = data.readlines()

        self.nData = len(lines)
        self.times = np.zeros(self.nData)
        self.position = np.zeros((self.nData,3))
        self.velocity = np.zeros((self.nData,3))
        self.acceleration = np.zeros((self.nData,3))
        self.xAxis = np.zeros((self.nData,3))
        self.yAxis = np.zeros((self.nData,3))
        self.zAxis = np.zeros((self.nData,3))

        idx = 0
        for line in lines:
            row = line.split()
            self.times[idx] = float(row[0])
            self.position[idx,0] = float(row[1])
            self.position[idx,1] = float(row[2])
            self.position[idx,2] = float(row[3])
            self.xAxis[idx,0] = float(row[4])
            self.xAxis[idx,1] = float(row[5])
            self.xAxis[idx,2] = float(row[6])
            self.yAxis[idx,0] = float(row[7])
            self.yAxis[idx,1] = float(row[8])
            self.yAxis[idx,2] = float(row[9])
            self.zAxis[idx,0] = float(row[10])
            self.zAxis[idx,1] = float(row[11])
            self.zAxis[idx,2] = float(row[12])
            idx += 1

        for i in range(1,self.nData-1):
            dt = 1.0/(self.times[i+1] - self.times[i-1])
            self.velocity[i] = (self.position[i+1] - self.position[i-1])*dt
            self.acceleration[i] = (self.position[i+1] - 2*self.position[i-1] + self.position[i-1])*dt*2
            if self.t_burnout > 0:
                pass


        self.plot_path()


    def plot_path(self):
        self.axes.clear()
        self.axes.plot(self.position[:,0],self.position[:,1],self.position[:,2],':k')
        maxz = np.amax(self.position)
        self.axes.set_zlim([0,maxz])
        self.axes.set_xlim([maxz*-0.5,maxz*0.5])
        self.axes.set_ylim([maxz*-0.5,maxz*0.5])
        

    def animate(self):
        nData = len(self.times)
        scale = 500
        
        x_axis, = self.axes.plot([0,scale*self.xAxis[0,0]],[0,scale*self.xAxis[0,1]],[0,scale*self.xAxis[0,2]],'r')
        y_axis, = self.axes.plot([0,scale*self.yAxis[0,0]],[0,scale*self.yAxis[0,1]],[0,scale*self.yAxis[0,2]],'g')
        z_axis, = self.axes.plot([0,scale*self.zAxis[0,0]],[0,scale*self.zAxis[0,1]],[0,scale*self.zAxis[0,2]],'b')
        #flame = 
        start_time = time.time()
        for i in range(nData):
            px = self.position[i,0]
            py = self.position[i,1]
            pz = self.position[i,2]
            x_axis.set_xdata([px,px + scale*self.xAxis[i,0]])
            x_axis.set_ydata([py,py + scale*self.xAxis[i,1]])
            x_axis.set_3d_properties([pz,pz + scale*self.xAxis[i,2]])
            y_axis.set_xdata([px,px + scale*self.yAxis[i,0]])
            y_axis.set_ydata([py,py + scale*self.yAxis[i,1]])
            y_axis.set_3d_properties([pz,pz + scale*self.yAxis[i,2]])
            z_axis.set_xdata([px,px + scale*self.zAxis[i,0]])
            z_axis.set_ydata([py,py + scale*self.zAxis[i,1]])
            z_axis.set_3d_properties([pz,pz + scale*self.zAxis[i,2]])
            self.figure.canvas.draw()
            self.figure.canvas.flush_events()
            time.sleep(max(start_time + self.times[i] - time.time(),0))
            print(self.times[i])
                

if __name__ == '__main__':
    app = App()
    app.mainloop()


