import os
import sys
import cv2
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from scipy.misc import imread

#maps = ['map', 'buildings', 'open', 'offices']
maps = ['buildings']


resolution = {'offices':0.08, 'buildings': 0.14, 'open': 0.15, 'map': 0.1}
values = {'offices':[800, 600], 'buildings': [1742, 1598], 'open': [800, 600], 'map':[681, 677]}
map_names = {'offices': 'offices.png', 'buildings': 'buildings.png', 'open': 'open.png', 'map': 'map.png'}

palette = {'WTS': '#ff7f7f', 'T*-RRT': '#ffb481', 'V-RRT': '#9cff86',
           'RRT': '#89d8ff', 'RRT*-first':'#efa7de', 'RRT*': '#9278dd'}

colors = {'nh': 'green', 'theta_star_rrt': 'blue', 'voronoi_rrt': 'fuchsia',
         'rrt': 'gold', 'rrt_star_first':'brown', 'rrt_star_last': 'red'}
names = {'nh': 'WTS', 'theta_star_rrt': 'Theta*-RRT', 'voronoi_rrt': 'Voronoi-RRT',
         'rrt': 'RRT', 'rrt_star_first':'RRT*-first', 'rrt_star_last': 'RRT*'}

#algorithms = ['rrt',  'rrt_star_last', 'theta_star_rrt', 'voronoi_rrt', 'nh']
#algorithms = ['rrt', 'rrt_star_last', 'nh']
#algorithms = ['theta_star_rrt', 'voronoi_rrt']
algorithms = ['rrt_star_last', 'rrt', 'nh']

def draw(m, c):
    params = {'legend.fontsize': 10}
    matplotlib.rcParams.update(params)

    filename = os.getcwd() +'/maps/' + map_names[m]
    img = imread(filename)
    r = resolution[m]
    left = 0
    right = (values[m][0] * r)
    bottom = 0
    top = (values[m][1] *  r)
    plt.ylim(ymax=top)
    plt.xlim(xmax=right)
    plt.imshow(img,zorder=0, extent=[left, right, bottom, top])

    for a in algorithms:
        path = get_path(m, a, c)
        plt.plot(path[1:,0], path[1:,1], color=colors[a], label=names[a], linewidth=2, zorder=1)

    #path1 = get_path_th(m, 'nh', c, 300, 0)
    #plt.plot(path1[1:,0], path1[1:,1], color=palette['RRT*'], linewidth=4, zorder=1)
    #path2 = get_path_th(m, 'nh', c, 300, 200)
    #plt.plot(path2[1:,0], path2[1:,1], color=colors['voronoi_rrt'], linewidth=1, zorder=1)
    #path3 = get_path_th(m, 'nh', c, 400, 240)
    #plt.plot(path3[1:,0], path3[1:,1], color=palette['RRT'], linewidth=3, zorder=2)
    #path4 = get_path_th(m, 'nh', c, 400, 300)
    #plt.plot(path4[1:,0], path4[1:,1], color=palette['RRT*-first'], linewidth=3, zorder=2)

    #plt.legend()
    #plt.show()
    plt.axis('off')
    plt.savefig(os.getcwd() + '/paths/' + m + '_' + c + '_bicycle.pdf')
    #plt.savefig('/home/reb/MasterThesis/chapters/pictures/ch6/paths/suchquality.pdf')
    plt.clf()
    plt.close()


def get_path(m, a, c):
    if m == 'map':
        x_origin = 31.3827
        y_origin = 27.814445
    else:
        x_origin = 25
        y_origin = 30

    prefix = m + '/' + a + '_' + m + '_' + c + '_'
    min_length = 10000000000000000
    #wtf = 0
    min_log = ''

    for i in range(0,50):
        if a == 'rrt_star_last':
            name = '/logs/bicycle/' + m + '/'+ 'rrt_star' + '_' + m + '_' + c + '_'+ str(i) + '_last.log'
        else:
            name = '/logs/bicycle/' + prefix + str(i) + '.log'
        log = open(os.getcwd() + name, 'r')
        log_lines = log.readlines()
        if(len(log_lines) > 5):
            if(len(log_lines) > 5):
                if(a == 'rrt_star_last' or a == 'rrt'):
                    length = float(log_lines[1].split()[1])
                    if length < 80:
                        log.close()
                        continue
                length = float(log_lines[1].split()[1])
                if(length < min_length):
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

    print min_log
    print min_length
    print '-----------------------------'
    return path

def get_path_th(m, a, c, l_max, l_min):
    if m == 'map':
        x_origin = 31.3827
        y_origin = 27.814445
    else:
        x_origin = 25
        y_origin = 30

    prefix = m + '/' + a + '_' + m + '_' + c + '_'
    min_length = 10000000000000000
    #wtf = 0
    min_log = ''

    for i in range(0,50):
        if a == 'rrt_star_last':
            name = '/logs/bicycle/' + m + '/'+ 'rrt_star' + '_' + m + '_' + c + '_'+ str(i) + '_last.log'
        else:
            name = '/logs/bicycle/' + prefix + str(i) + '.log'
        log = open(os.getcwd() + name, 'r')
        log_lines = log.readlines()
        if(len(log_lines) > 5):
            if(len(log_lines) > 5):
                if(a == 'rrt_star_last' or a == 'rrt'):
                    length = float(log_lines[1].split()[1])
                    if length < 80:
                        log.close()
                        continue
                length = float(log_lines[1].split()[1])
                if(length < min_length):

                    if (length < l_max and length > l_min):
                        min_length = length
                        min_log = name


        log.close()

    log = open(os.getcwd() + min_log, 'r')
    print min_log
    print min_length
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
    return path


def draw_one(m, c, i):
    filename = os.getcwd() +'/maps/' + map_names[m]
    img = imread(filename)
    r = resolution[m]
    left = 0
    right = (values[m][0] * r)
    bottom = 0
    top = (values[m][1] *  r)
    plt.ylim(ymax=top)
    plt.xlim(xmax=right)
    plt.imshow(img,zorder=0, extent=[left, right, bottom, top])

    path = get_path(m, 'nh', c)
    plt.plot(path[1:,0], path[1:,1], color=colors['nh'], label=names['nh'], linewidth=1, zorder=1)

    log = open(os.getcwd() + '/logs/bicycle/'+ m + 'rrt_' + m + '_'+ c +'_'+ i + '.log', 'r')
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
    path1 = np.asarray(points, dtype = float)
    plt.plot(path1[1:,0], path1[1:,1], color=colors['rrt'], label=names['rrt'], linewidth=1, zorder=1)
    plt.show()


if __name__ == '__main__':
    draw('buildings', '3')

    #for m in maps:
        #for i in range(0,1):
            #draw(m, str(3))
