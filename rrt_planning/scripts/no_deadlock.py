import os
import sys
import time
import gflags
import subprocess

from joblib import Parallel, delayed

gflags.DEFINE_integer('n_jobs', 8, 'number of parallel experiments')
gflags.DEFINE_string('deadline', '300', 'deadline (in seconds)')
gflags.DEFINE_string('model', 'differentialDrive', 'kinematic model')

algorithms = ['nh', 'nh_l2', 'rrt', 'rrt_star', 'theta_star_rrt', 'voronoi_rrt']
maps = ['map', 'offices']

def experiment(a, c, row, i, m):
    print ''
    print '*************************************************'
    it = row + '_' + i
    ns = a + '_' + m + '_' + it
    os.system('rosparam load config/' + a + '.yaml ' + ns)
    os.system('rosparam load config/' + gflags.FLAGS.model + '.yaml ' + ns)
    os.system('rosparam load config/' + 'global_costmap_params.yaml ' + ns)
    os.system('rosparam load config/' + 'costmap_common_params.yaml ' + ns + '/global_costmap')
    os.system('rosrun rrt_planning experiment ' +
              ' ' + a +
              ' ' + m +
              ' ' + c +
              ' ' + it +
              ' ' + gflags.FLAGS.deadline +
              ' ' + os.getcwd() + '/logs/' + m + '/')


def run(configurations, alg, m):
    roscore = subprocess.Popen('roscore')
    time.sleep(1)
    origWD = os.getcwd()
    print 'launching map server'
    map_dir = "maps/" + m + ".yaml"
    map_server = subprocess.Popen(['rosrun', 'map_server', 'map_server', map_dir])
    time.sleep(1)
    print 'launching static tf'
    tf = subprocess.Popen(['rosrun', 'tf', 'static_transform_publisher',
                            '1','0','0','0','0','0','1','base_link','map','100'])
    time.sleep(1)

    Parallel(n_jobs=gflags.FLAGS.n_jobs)(delayed(experiment)
                                        (alg, conf, str(configurations.index(conf)), str(i), m)
                                         for conf in configurations
                                         for i in range(0,50)
                                         )
    print alg + ' out'
    subprocess.Popen.kill(roscore)
    subprocess.Popen.kill(map_server)
    subprocess.Popen.kill(tf)
    os.system('killall -9 rosmaster')
    os.system('killall -9 rosout')
    time.sleep(1)

if __name__ == '__main__':
    argv = gflags.FLAGS(sys.argv)
    
    for m in maps:
        filename = os.getcwd() + "/data/" + m + ".exp";
        with open(filename) as f:
            configurations = f.read().splitlines()
        for alg in algorithms:
            run(configurations, alg, m)
