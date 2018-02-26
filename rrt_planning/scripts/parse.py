import os
import pandas as pd

#compare everything
algorithms = ['nh', 'rrt', 'rrt_star_first', 'rrt_star_last', 'theta_star_rrt', 'voronoi_rrt']
maps = ['open', 'map', 'buildings', 'offices']


#compare just some
#algorithms = ['rrt_star_last', 'rrt_star_first']
#algorithms =['nh', 'rrt', 'rrt_star_first', 'rrt_star_last', 'theta_star_rrt']
#maps = ['open']

max_conf = 5
max_run = 50

def parse_logs(m):
    wd = os.getcwd()
    compare = open(wd + '/results/compare/' + m + '.csv', 'w')
    compare.writelines('conf,algorithm,run,length,time,roughness' + '\n')
    for alg in algorithms:
        for conf in range(0,max_conf):
            for run in range(0,max_run):
                if alg == 'rrt_star_first':
                    if os.path.exists(wd + '/logs/'+ m + "/" + 'rrt_star_' + m + '_' + str(conf) + '_' + str(run) +'_first.log'):
                        log = open(wd + '/logs/'+ m + "/" + 'rrt_star_' + m + '_' + str(conf) + '_' + str(run) +'_first.log', 'r')
                    else:
                        continue
                elif alg == 'rrt_star_last':
                    if os.path.exists(wd + '/logs/'+ m + "/" + 'rrt_star_' + m + '_' + str(conf) + '_' + str(run) +'_last.log'):
                        log = open(wd + '/logs/'+ m + "/" + 'rrt_star_' + m + '_' + str(conf) + '_' + str(run) +'_last.log', 'r')
                    else:
                        continue
                else:
                    log = log = open(wd + '/logs/'+ m + "/" + alg +'_' + m + '_' + str(conf) + '_' + str(run) +'.log', 'r')
                log_lines = log.readlines()[1:]
                results = [str(conf), alg, str(run), '0', '0', '0']
                for line in log_lines:
                    if 'NO_PATH_FOUND_WITHIN_DEADLINE' in line:
                        results[4] = '300'
                        break
                    if 'FAILED_TO_FIND_PLAN' in line:
                        break
                    else:
                        s = line.split()
                        if s[0] == 'length':
                            results[3] = s[1]
                        elif s[0] == 'time':
                            results[4] = s[1]
                        elif s[0] == 'roughness':
                            results[5] = s[1]
                log.close()
                compare.writelines(results[0] + ',' + results[1] + ',' + results[2] + ',' +
                                   results[3] + ',' + results[4] + ',' + results[5] +  '\n')

    compare.close()

def describe(m):
    wd = os.getcwd()
    compare = open(wd + '/results/compare/' + m + '_shenanigans.csv', 'w')
    compare.writelines('conf,alg,mean_length,std_length,mean_time,std_time,mean_roughness,std_roughness,success_rate' + '\n')
    df = pd.read_csv(wd + '/results/compare/' + m + '.csv')
    for conf in range(0,max_conf):
        for alg in algorithms:
            data = df[(df.algorithm == alg)]
            data = data[(data.conf == conf)]
            results = [str(conf), alg, '0', '0', '0', '0', '0', '0', '0']
            success = data[(data.length!=0)]

            results[2] = success.length.mean()
            results[3] = success.length.std()
            results[4] = success.time.mean()
            results[5] = success.time.std()
            results[6] = success.roughness.mean()
            results[7] = success.roughness.std()
            results[8] = float(success.shape[0]) / float(data.shape[0]) * 100

            compare.writelines(str(results[0]) + ',' + str(results[1]) + ',' + str(results[2]) + ',' +
                               str(results[3]) + ',' + str(results[4]) + ',' + str(results[5]) + ',' +
                               str(results[6]) + ',' + str(results[7]) + ',' + str(results[8]) + '\n')

    compare.close()

if __name__ == '__main__':
    for m in maps:
        parse_logs(m)
