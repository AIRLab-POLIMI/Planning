import os
import sys
import time
import gflags
import subprocess

from joblib import Parallel, delayed

gflags.DEFINE_integer('n_jobs', -1, 'number of parallel experiments')
gflags.DEFINE_string('deadline', '300', 'deadline (in seconds)')
gflags.DEFINE_integer('n_exp', 1, 'number of experiments')
gflags.DEFINE_string('env_name', 'map', 'environment name')
gflags.DEFINE_string('model', 'differentialDrive', 'kinematic model')

maps = ['map']
#algorithms = ['nh', 'rrt', 'rrt_star', 'theta_star_rrt', 'voronoi_rrt']
algorithms = ['nh']

def experiment(a, c, i):
    print ''
    print '*************************************************'
    print a, c, i
    ns = a + '_' + 'map' + '_' + i
    os.system('rosparam load config/' + a + '.yaml ' + ns)
    os.system('rosparam load config/' + gflags.FLAGS.model + '.yaml ' + ns)
    os.system('rosparam load config/' + 'global_costmap_params.yaml ' + ns)
    os.system('rosparam load config/' + 'costmap_common_params.yaml ' + ns + '/global_costmap')
    os.system('rosrun rrt_planning experiment ' +
              ' ' + a +
              ' ' + gflags.FLAGS.env_name +
              ' ' + c +
              ' ' + i +
              ' ' + gflags.FLAGS.deadline +
              ' ' + os.getcwd() + '/logs/')
    print 'node has died'


if __name__ == '__main__':
    argv = gflags.FLAGS(sys.argv)
    filename = os.getcwd() + "/data/" + "map" + ".exp";
    with open(filename) as f:
        configurations = f.read().splitlines()
    print configurations
    roscore = subprocess.Popen('roscore')
    time.sleep(1)
    origWD = os.getcwd()
    print 'launching map server'
    map_dir = os.getcwd() + "maps/map.yaml"
    map_server = subprocess.Popen(['rosrun', 'map_server', 'map_server', 'maps/map.yaml'])
    time.sleep(1)
    print 'launching static tf'
    tf = subprocess.Popen(['rosrun', 'tf', 'static_transform_publisher',
                            '1','0','0','0','0','0','1','base_link','map','100'])
    time.sleep(1)

    Parallel(n_jobs=gflags.FLAGS.n_jobs)(delayed(experiment)
                                        (alg, conf, str(configurations.index(conf)))
                                        for alg in algorithms
                                        for conf in configurations
                                        )
    print 'magic is real'
    subprocess.Popen.kill(map_server)
    subprocess.Popen.kill(tf)
    subprocess.Popen.kill(roscore)
    os.system('killall -9 rosmaster')
