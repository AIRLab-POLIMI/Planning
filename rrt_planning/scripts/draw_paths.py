import os
import sys
import cv2
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from scipy.misc import imread

save_path = '/home/dave/Planning/plots/'

models = ['differential_drive', 'bicycle']
maps = ['floor', 'street', 'open', 'offices']

resolution = {'offices':0.08, 'street': 0.14, 'open': 0.15, 'floor': 0.1}
values = {'offices':[800, 600], 'street': [1742, 1598],
          'open': [800, 600], 'floor':[681, 677]}
map_names = {'offices': 'offices.png', 'street': 'buildings.png',
             'open': 'b-w-open.png', 'floor': 'map.bmp'}

palette = {'WTS': '#ff7f7f', 'T*-RRT': '#ffb481', 'V-RRT': '#9cff86',
           'RRT': '#89d8ff', 'RRT*-first':'#efa7de', 'RRT*': '#9278dd'}

names = {'wts': 'WTS', 'theta_star_rrt': 'T*-RRT', 'voronoi_rrt': 'V-RRT',
         'rrt': 'RRT', 'rrt_star_first':'RRT*-first', 'rrt_star_last': 'RRT*'}

algorithms = ['rrt',  'rrt_star_last', 'theta_star_rrt', 'voronoi_rrt', 'wts']


def draw(k, m, c, savefig=False):
    params = {'legend.fontsize': 10}
    matplotlib.rcParams.update(params)

    filename = os.getcwd() + '/maps/' + map_names[m]
    img = imread(filename, mode='RGB')
    r = resolution[m]
    left = 0
    right = (values[m][0] * r)
    bottom = 0
    top = (values[m][1] * r)
    plt.ylim(ymax=top)
    plt.xlim(xmax=right)
    plt.imshow(img,zorder=0, extent=[left, right, bottom, top])

    for a in algorithms:
        path = get_path(k, m, a, c)
        plt.plot(path[1:,0], path[1:,1], color=palette[names[a]],
                 label=names[a], linewidth=2, zorder=1)

    plt.legend(fontsize=8)

    plt.axis('off')
    if savefig:
        plt.savefig(save_path + '/paths/' + k + '/' + m + '_' + c + '.pdf')


def get_path(k, m, a, c):
    if m == 'floor':
        x_origin = 31.3827
        y_origin = 27.814445
    else:
        x_origin = 25
        y_origin = 30

    prefix = k + '/' + m + '/' + a + '_' + c + '_'
    min_length = 10000000000000

    if (a != 'wts'):
        min_length = 0

    min_log = ''

    for i in range(0,50):
        if a == 'rrt_star_last':
            name = '/logs/' + k + '/' + m + '/' + 'rrt_star' + '_' + c + '_' + \
                   str(i) + '_last.log'
        else:
            name = '/logs/' + prefix + str(i) + '.log'

        log = open(os.getcwd() + name, 'r')
        log_lines = log.readlines()
        if(len(log_lines) > 5):
             length = float(log_lines[1].split()[1])
             if (a == 'wts'):
                 if(length < min_length):
                     min_length = length
                     min_log = name
             elif (a == 'rrt'):
                 if(length > min_length):
                     min_length = length
                     min_log = name
             else:
                 if(length > min_length and length < 400):
                     min_length = length
                     min_log = name


        log.close()

    log = open(os.getcwd() + min_log, 'r')
    log_lines = log.readlines()
    np.set_printoptions(suppress=True)
    lines = log_lines[5:]
    start = log_lines[4].split('_')
    points = []
    points.append([float(start[0]), float(start[1])])
    for line in lines:
        c = line.split('_')
        point = [float(c[0]) + x_origin , float(c[1]) + y_origin]
        if point != points[-1]:
            points.append(point)
    log.close()
    path = np.asarray(points, dtype = float)

    print(min_log)
    print(min_length)
    print('-----------------------------')
    return path


if __name__ == '__main__':
    show = False
    savefig = True
    for k in models:
        for m in maps:
            for i in range(0,5):
                draw(k, m, str(i), savefig)
                if show:
                    plt.show()
                plt.close('all')
