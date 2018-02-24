import os
import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

algorithms = ['rrt_star']
maps = ['map']


def roughness(alg, m , conf, run):
    wd = os.getcwd()
    log = open(wd + '/logs/map/'+ alg + '_' + m + '_' + conf + '_' + run +'_first.log', 'r')
    log_final = open(wd + '/logs/map/'+ alg + '_' + m + '_' + conf + '_' + run +'_last.log', 'r')

    #first path
    log_lines = log.readlines()
    if len(log_lines) < 5:
        return
    np.set_printoptions(suppress=True)
    length = float(log_lines[1].split()[1])
    r = float(log_lines[3].split()[1])
    lines = log_lines[5:]
    start = log_lines[4].split('_')
    points = []
    points.append([float(start[0]), float(start[1])])
    for line in lines:
        c = line.split('_')
        point = [float(c[0]), float(c[1])]
        if point != points[-1]:
            points.append(point)
    path = np.asarray(points, dtype = float)

    #final_path
    log_lines_final = log_final.readlines()
    np.set_printoptions(suppress=True)
    length_final = float(log_lines_final[1].split()[1])
    if(length_final == length):
        return
    r_final = float(log_lines_final[3].split()[1])
    lines_final = log_lines_final[5:]
    start_final = log_lines_final[4].split('_')
    points_final = []
    points_final.append([float(start_final[0]), float(start_final[1])])
    for line_final in lines_final:
        c = line_final.split('_')
        point_final = [float(c[0]), float(c[1])]
        if point_final != points_final[-1]:
            points_final.append(point_final)
    path_final = np.asarray(points_final, dtype = float)

    plt.plot(path[1:,0], path[1:,1], color='green', label='first')
    plt.plot(path_final[1:,0], path_final[1:,1], color='red', label='last')

    plt.show()


if __name__ == '__main__':
    for alg in algorithms:
        for m in maps:
            for conf in range(0,5):
                for i in range(0,50):
                    roughness(alg, m, str(conf), str(i))
