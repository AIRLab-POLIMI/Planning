import pandas as pd
import os

#algorithms = ['nh', 'forward_nh', 'theta_star_rrt']
#algorithms = ['theta_star_rrt', 'nh_s2_p5', 'nh_s2_p1', 'nh_s3_p5', 'nh_s3_p1',
              #'forward_nh_s2_p5', 'forward_nh_s2_p1', 'forward_nh_s3_p5', 'forward_nh_s3_p1']
algorithms = ['theta_star_rrt', 'nh_s2_p1', 'nh_s2_p5', 'nh_s3_p1', 'nh_s3_p5']

def parse():
    wd = os.getcwd()
    compare = open(wd + '/logs/compare/results/outdoor_shenanigans.csv', 'w')
    compare.writelines('conf,alg,mean_length,std_length,mean_time,std_time,mean_roughness,std_roughness,success_rate,mean_iterations' + '\n')
    for i in range(0,10):
        for alg in algorithms:
            df = pd.read_csv(wd + '/logs/compare/open/' + alg + '_' + str(i) + '_compare.csv')
            results = [str(i), alg, '0', '0', '0', '0', '0', '0', '0', '0']
            success = df[(df.length!=0)]

            results[2] = success.length.mean()
            results[3] = success.length.std()
            results[4] = success.time.mean()
            results[5] = success.time.std()
            results[6] = success.roughness.mean()
            results[7] = success.roughness.std()
            results[8] = float(success.shape[0]) / float(df.shape[0]) * 100
            results[9] = success.iterations.mean()

            compare.writelines(str(results[0]) + ',' + str(results[1]) + ',' + str(results[2]) + ',' +
                               str(results[3]) + ',' + str(results[4]) + ',' + str(results[5]) + ',' +
                               str(results[6]) + ',' + str(results[7]) + ',' + str(results[8]) + ',' +
                               str(results[9]) + '\n')

    compare.close()


if __name__ == '__main__':
    parse()
