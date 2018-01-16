import os


def parse():
    wd = os.getcwd()
    compare = open(wd + '/logs/compare.csv', 'w')
    compare.writelines('NH_l,NH_t,FNH_t,FH_l' + '\n')
    for i in range(0,100):
        nh = open(wd + '/logs/nh_map_' + str(i) + '.log', 'r')
        nh_lines = nh.readlines()[1:]
        results = ['0', '0', '0', '0']
        for line in nh_lines:
            if 'NO_PLAN_FOUND_WITHIN_DEADLINE' in line:
                break
            else:
                s = line.split()
                if s[0] == 'length':
                    results[0] = s[1]
                elif s[0] == 'time':
                    results[1] = s[1]
        nh.close()
        fnh = open(wd + '/logs/forward_nh_map_' + str(i) + '.log', 'r')
        fnh_lines = fnh.readlines()[1:]
        for line in fnh_lines:
            if 'NO_PLAN_FOUND_WITHIN_DEADLINE' in line:
                break
            else:
                s = line.split()
                if s[0] == 'length':
                    results[3] = s[1]
                elif s[0] == 'time':
                    results[2] = s[1]
        fnh.close()
        compare.writelines(results[0] + ',' + results[1] + ',' + results[2] + ',' + results[3] + '\n')

    compare.close()


if __name__ == '__main__':
    parse()
