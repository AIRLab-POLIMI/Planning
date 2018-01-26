import os

algorithms = ['forward_nh']

def parse():
    wd = os.getcwd()
    for alg in algorithms:
        compare = open(wd + '/logs/' + alg + '_compare.csv', 'w')
        compare.writelines('length,time,roughness' + '\n')
        for i in range(0,50):
            nh = open(wd + '/logs/'+ alg +'_map_' + str(i) + '.log', 'r')
            nh_lines = nh.readlines()[1:]
            results = ['0', '0', '0']
            for line in nh_lines:
                if 'NO_PATH_FOUND_WITHIN_DEADLINE' in line:
                    results[1] = '300'
                    break
                if 'FAILED_TO_FIND_PLAN' in line:
                    break
                else:
                    s = line.split()
                    if s[0] == 'length':
                        results[0] = s[1]
                    elif s[0] == 'time':
                        results[1] = s[1]
                    elif s[0] == 'roughness':
                        results[2] = s[1]
            nh.close()
            compare.writelines(results[0] + ',' + results[1] + ',' + results[2] + '\n')

    compare.close()


if __name__ == '__main__':
    parse()
