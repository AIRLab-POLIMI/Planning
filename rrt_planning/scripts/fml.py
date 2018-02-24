import os

#algorithms = ['nh', 'nh_l2', 'rrt', 'theta_star_rrt', 'voronoi_rrt', 'rrt_star']
algorithms = ['voronoi_rrt']
maps = ['map', 'offices', 'buildings', 'open']
#maps =['map']

conf = 5
run = 50

def wtf(m):
    wd = os.getcwd() + '/logs/voronoi/voronoi/' + m + '/'
    logs = os.listdir(wd)
    data = {'nh':0, 'nh_l2':0, 'rrt':0, 'theta_star_rrt':0, 'voronoi_rrt':0, 'rrt_star': 0}
    for alg in algorithms:
        for c in range(0, conf):
            for f in logs:
                if f.startswith(alg + '_' + m + '_' + str(c)):
                    data[alg] += 1
    print 'map: ' + m
    print data




if __name__ == '__main__':
    for m in maps:
        wtf(m)
