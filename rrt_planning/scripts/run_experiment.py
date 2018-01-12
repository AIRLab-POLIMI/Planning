import os
import sys
import gflags

from joblib import Parallel, delayed

gflags.DEFINE_integer('n_jobs', -1, 'number of parallel experiments')
#gflags.DEFINE_string('deadline', '60', 'deadline (in seconds)')
gflags.DEFINE_integer('n_exp', 1, 'number of experiments')
gflags.DEFINE_string('env_name', 'map', 'environment name')

maps = ['map']
algorithms = ['nh', 'rrt', 'rrt_star', 'theta_star_rrt', 'voronoi_rrt']

if __name__ == '__main__':
    argv = gflags.FLAGS(sys.argv)
    with open(filename) as f:
        configurations = f.read().splitlines()

    os.system('roscore')
    print 'launching map server'
    os.system('rosrun map_server map_server $(find rrt_planning)/maps/'
               + env_name + '.yaml' )
    print 'launching static tf'
    os.system('rosrun tf static_transform_publisher 1 0 0 0 0 0 1 base_link map 100')

    Parallel(n_jobs=gflags.FLAGS.n_jobs)(delayed(os.system)
                                        ('rosrun rrt_planning experiment ' + os.getcwd() +
                                         ' ' + alg
                                         ' ' + env_name
                                         ' ' + conf
                                         ' ' + conf.index(value))
                                         for alg in algorithms
                                         for conf in confingurations
