import os
import sys
import time
import gflags
import subprocess

from joblib import Parallel, delayed

gflags.DEFINE_integer('n_jobs', 2, 'number of parallel experiments')
gflags.DEFINE_string('deadline', '300', 'deadline (in seconds)')
gflags.DEFINE_string('model', 'differentialDrive', 'kinematic model')

#algorithms = ['nh', 'rrt', 'theta_star_rrt', 'rrt_star']
algorithms = ['nh']
maps = ['buildings']
#maps = ['buildings']

def experiment(a, c, row, i, m):
    print ''
    print '*************************************************'
    it = row + '_' + i
    ns = a + '_' + m + '_' + it
    if m == 'open' or m == 'buildings':
        os.system('rosparam load config/outdoor/' + a + '.yaml ' + ns)
        os.system('rosparam load config/outdoor/' + 'costmap_common_params.yaml ' + ns + '/global_costmap')
    elif m == 'offices' or m == 'map':
        os.system('rosparam load config/indoor/' + a + '.yaml ' + ns)
        os.system('rosparam load config/indoor/' + 'costmap_common_params.yaml ' + ns + '/global_costmap')

    os.system('rosparam load config/' + gflags.FLAGS.model + '.yaml ' + ns)
    os.system('rosparam load config/' + 'global_costmap_params.yaml ' + ns)
    os.system('rosrun rrt_planning experiment ' +
              ' ' + a +
              ' ' + m +
              ' ' + c +
              ' ' + it +
              ' ' + gflags.FLAGS.deadline +
              ' ' + os.getcwd() + '/logs/wtf' +'/')


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
    conf = configurations[1]
    Parallel(n_jobs=gflags.FLAGS.n_jobs)(delayed(experiment)
                                        (alg, conf, str(configurations.index(conf)), str(i), m)
                                         #for conf in configurations
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
