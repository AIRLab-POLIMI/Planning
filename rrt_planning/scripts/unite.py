import os
import pandas as pd

algorithms = ['forward_nh']

def parse(alg):
    wd = os.getcwd()
    compare = open(wd + '/logs/' + alg + '_values.csv', 'w')
    compare.writelines('length,time,roughness,success_rate' + '\n')
    for i in range(0,1):
        nh = pd.read_csv(wd + '/logs/' + alg + '_compare.csv')
        results = ['0', '0', '0', '0']
        nh_failures = nh[(nh.length == 0)]
        nh_success = nh[(nh.length!=0)]
        
        success_rate = float(nh_success.shape[0]) / float(nh.shape[0]) * 100
        #success_rate = nh_success.shape[0] + 1
        
        mean_l = nh_success.length.mean()
        mean_t = nh_success.time.mean()
        mean_r = nh_success.roughness.mean()
        
        results[0] = mean_l
        results[1] = mean_t
        results[2] = mean_r
        results[3] = success_rate
        
        compare.writelines(str(results[0]) + ',' + str(results[1]) + ',' + str(results[2]) + ',' + str(results[3]) + '\n')

    compare.close()


if __name__ == '__main__':
    for alg in algorithms:
        parse(alg)
