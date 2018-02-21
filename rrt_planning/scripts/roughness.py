import os
import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

algorithms = ['theta_star_rrt']
maps = ['map']


def roughness(alg, m , conf, run):
    wd = os.getcwd()
    log = open(wd + '/logs/'+ alg + '_' + m + '_' + conf + '_' + run +'.log', 'r')
    log_lines = log.readlines()
    np.set_printoptions(suppress=True)
    #path = np.zeros([len(log_lines) - 4, 2], dtype=float)
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

    #first derivative
    dx = np.gradient(path[:, 0])
    dy = np.gradient(path[:, 1])

    #second derivative
    d2x = np.gradient(dx)
    d2y = np.gradient(dy)

    #curvature
    k = np.abs(d2x * dy - dx * d2y) / ((dx * dx + dy *dy)** 1.5)
    dk = np.gradient(k)

    new_r = 0.0
    new_dr = 0.0

    for i in k:
        new_r = new_r + np.abs(pow(i / length, 2))
    for di in dk:
        new_dr = new_dr + np.abs(pow(di /length, 2))


    print 'conf ' + conf +', run ' + run + ', length ' + str(length)
    print 'points: ' + str(len(lines)) + ' no orientation: ' + str(len(points))
    print 'discrete r: ' + str(r)
    print 'curvature: ' + str(new_r)
    print 'derivative: ' + str(new_dr)
    print '***********************************'


    plt.plot(path[1:,0], path[1:,1])
    #plt.show()


if __name__ == '__main__':
    for alg in algorithms:
        for m in maps:
            for conf in range(0,9):
                for i in range(0,50):
                    roughness(alg, m, str(conf), str(i))
