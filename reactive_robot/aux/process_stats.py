#!/usr/bin/env python

import os
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def calculate_averages():
    stats_dir = "/home/mfc/Documents/ROBO/ROBO2021/reactive_robot/stats/wall_0.6/"
    err_files = os.listdir(stats_dir)
    for file in err_files:
        csv = pd.read_csv(stats_dir + file)
        x = list(csv['time'])
        y = list(csv[' error'])
        fig, ax = plt.subplots()
        ax.plot(x, y)
        ax.set_xticks(np.arange(0, 45, 5))
        # plt.show()
        fig.savefig("/home/mfc/Documents/ROBO/ROBO2021/reactive_robot/stats/img/wall_0.6/plot_" + file  + ".png")

        print(csv[' error'].mean())

if __name__ == '__main__':
    calculate_averages()