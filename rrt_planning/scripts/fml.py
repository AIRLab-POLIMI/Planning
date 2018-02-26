import os

#algorithms = ['nh', 'nh_l2', 'rrt', 'theta_star_rrt', 'voronoi_rrt', 'rrt_star']
algorithms = ['voronoi_rrt']
maps = ['map', 'offices', 'buildings', 'open']
#maps =['map']

conf = 5
run = 50


def count(m):
    wd = os.getcwd() + '/logs/' + m + '/'
    logs = os.listdir(wd)
    data = {'nh':0, 'nh_l2':0, 'rrt':0, 'theta_star_rrt':0, 'voronoi_rrt':0, 'rrt_star': 0}
    for alg in algorithms:
        for c in range(0, conf):
            for f in logs:
                if f.startswith(alg + '_' + m + '_' + str(c)):
                    data[alg] += 1
    print 'map: ' + m
    print data

def success_rate(m):
    if m == 'open' or m == 'buildings':
        max_run = 100
    else:
        max_run = 125

    wd = os.getcwd() + '/logs/' + m + '/'
    data = {m + '_0' : 0, m + '_1' : 0, m + '_2' : 0, m + '_3': 0, m + '_4' : 0}
    for c in range(0, conf):
        for i in range(0, run):
            f = open(wd + 'rrt_star_' + m + '_' + str(c) + '_' + str(i) +'_first.log', 'r')
            log_lines = f.readlines()
            if(len(log_lines) > 5):
                data[m + '_' + str(c)] += 1

    boost = os.getcwd() + '/logs/rrt_star/' + m + '/'
    data_boost = {m + '_0' : 0, m + '_1' : 0, m + '_2' : 0, m + '_3': 0, m + '_4' : 0}
    for c in range(0, conf):
        for i in range(run, max_run):
            f = open(boost + 'rrt_star_' + m + '_' + str(c) + '_' + str(i) +'_first.log', 'r')
            log_lines = f.readlines()
            if(len(log_lines) > 5):
                 data_boost[m + '_' + str(c)] += 1

    success_boost = {m + '_0' : 0.0, m + '_1' : 0.0, m + '_2' : 0.0, m + '_3': 0.0, m + '_4' : 0.0}
    for c in range(0,conf):
        runs = float(data[m + '_' + str(c)]) + float(data_boost[m + '_' + str(c)])
        success_boost[m + '_' + str(c)] = runs/50.0 * 100.0

    print '*************************'
    print data
    print data_boost
    print ' '
    print 'possible to boost up to: '
    print success_boost
    print ' '


if __name__ == '__main__':
    for m in maps:
        success_rate(m)
