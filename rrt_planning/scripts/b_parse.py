import os
import pandas as pd
import numpy as np
import math

#compare everything
algorithms = ['nh', 'theta_star_rrt', 'voronoi_rrt', 'rrt', 'rrt_star_first', 'rrt_star_last']
maps = ['open', 'map', 'buildings', 'offices']
#maps = ['open']

names = {'nh': 'WTS', 'theta_star_rrt': 'T*-RRT', 'voronoi_rrt': 'V-RRT',
         'rrt': 'RRT', 'rrt_star_first':'RRT*-first', 'rrt_star_last': 'RRT*'}

max_conf = 5
max_run = 50

def parse_logs(m):
    wd = os.getcwd()
    compare = open(wd + '/results/compare/' + m + '_bicycle.csv', 'w')
    compare.writelines('conf,algorithm,run,length,time,roughness,restarts' + '\n')
    for alg in algorithms:
        for conf in range(0,max_conf):
            for run in range(0,max_run):
                if alg == 'rrt_star_first':
                    log = open(wd + '/logs/bicycle/'+ m + "/" + 'rrt_star_' + m + '_' + str(conf) + '_' + str(run) +'_first.log', 'r')
                elif alg == 'rrt_star_last':
                    log = open(wd + '/logs/bicycle/'+ m + "/" + 'rrt_star_' + m + '_' + str(conf) + '_' + str(run) +'_last.log', 'r')
                else:
                    log = log = open(wd + '/logs/bicycle/'+ m + "/" + alg +'_' + m + '_' + str(conf) + '_' + str(run) +'.log', 'r')

                log_lines = log.readlines()
                points = []
                results = [str(conf), names[alg], str(run), '0', '0', '0']
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
                            if alg == 'voronoi_rrt':
                                if m == 'map':
                                    results[4] = str(float(results[4]) - 0.084)
                                elif m == 'offices':
                                    results[4] = str(float(results[4]) - 0.055)
                                elif m == 'open':
                                    results[4] = str(float(results[4]) - 0.028)
                                elif m == 'buildings':
                                    results[4] = str(float(results[4]) - 0.060)
                        elif s[0] == 'roughness':
                            results[5] = s[1]
                            for l in log_lines[4:]:
                                c = l.split('_')
                                point = float(c[2])
                                points.append(point)

                            raw_theta = np.asarray(points)
                            #compute delta of angles
                            alpha = []
                            for i in range(1, len(raw_theta)):
                                theta = float(np.abs(raw_theta[i] - raw_theta[i-1]))
                                if theta >= np.pi:
                                    theta = float(2*np.pi - theta)
                                alpha.append(theta)

                            #compute smoothness
                            roughness = 0.0
                            for i in alpha:
                                roughness = roughness + pow(i,2)

                            k = roughness / len(raw_theta)
                            results[5] = str(k)
                            break


                log.close()
                if alg == 'nh' and (m == 'open' or m == 'buildings'):
                    fuck_this_shit = open(wd + '/logs/bicycle/'+ m + "/" + alg +'_' + m + '_' + str(conf) + '_' + str(run) +'_restarts.log', 'r')
                    muda = fuck_this_shit.readlines()
                    compare.writelines(results[0] + ',' + results[1] + ',' + results[2] + ',' +
                                       results[3] + ',' + results[4] + ',' + results[5] + '\n')

                else:
                    compare.writelines(results[0] + ',' + results[1] + ',' + results[2] + ',' +
                                       results[3] + ',' + results[4] + ',' + results[5] +  '\n')


    compare.close()

def table(m):
    #reordering
    if m == 'map':
        order = [0,2,1,3,4]
        first = 0
        esp = 2
    elif m == 'offices':
        order = [1,4,2,0,3]
        first = 1
        esp = 1
    elif m == 'open':
        order = [0,4,2,3,1]
        first = 0
        esp = 2
    elif m == 'buildings':
        order = [2,1,0,4,3]
        first = 2
        esp = 2

    #query flag
    #query = 'query'
    query = 'q'


    #precisions
    l_p = '.2f'
    m_p = '.3f'
    h_p = '.4f'

    algorithm = ['nh', 'theta_star_rrt', 'voronoi_rrt', 'rrt', 'rrt_star_first','rrt_star_last']
    names = {'nh': 'WTS', 'theta_star_rrt': 'T*-RRT', 'voronoi_rrt': 'V-RRT',
             'rrt': 'RRT', 'rrt_star_first':'RRT*', 'rrt_star_last': 'RRT*-last'}
    remap ={'offices': 'Offices', 'map': 'Floor', 'open' : 'MOUT site', 'buildings': 'Sesto'}
    latex = {'offices': 'offices', 'map': 'floor', 'open' : 'mout', 'buildings': 'sesto'}
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + m + '_bicycle.csv')
    tab = open('/home/reb/MasterThesis/chapters/tables/FUKbi-' + latex[m] + '.tex', 'w')
    tab.writelines('\\begin{table}[!h]' + '\n' + '\\begin{center}' + '\\begin{small}' + '\n' +
                   '\\begin{tabular}{c|l|l r|l r|l r|r}')
    for conf in order:
        if conf == first:
            tab.writelines('$\mathbf{q_i}$ & \\textbf{Planner} & \multicolumn{2}{c|}{$\mathbf{t[s]}$} & \multicolumn{2}{c|}{$\mathbf{l[m]}$} &' +
                           '\multicolumn{2}{c|}{$\mathbf{\kappa}$ $\mathbf{[10^{-' + str(esp)+ '}rad^2]}$} & {$\mathbf{s_r}$} \\\\ \hline' + '\n')

        for a in algorithm:
            data = df[(df.algorithm == a)]
            data = data[(data.conf == conf)]
            results = ['0', '0', '0', '0', '0', '0', '0']
            success = data[(data.length!=0)]
            n = int(success.shape[0])

            if a == 'rrt_star_last':
                results[0] = '-'
                results[1] = '-'
            else:
                results[0] = success.time.mean()
                std_time = 1.96 * (success.time.std() / math.sqrt(n))
                results[1] = std_time
            results[2] = success.length.mean()
            std_length = 1.96 * (success.length.std() / math.sqrt(n))
            results[3] = std_length
            results[4] = success.roughness.mean() * (pow(10, esp))
            std_r = 1.96 * (success.roughness.std() / math.sqrt(n))
            results[5] = std_r * (pow(10, esp))
            if data.shape[0] == 0:
                print m
                print a
                print conf
            results[6] = float(success.shape[0]) / float(data.shape[0]) * 100
            results[6] = int(results[6])



            delimiter = '&'
            it = order.index(conf)
            if a == 'nh' and query == 'q':
                delimiter = '\multirow{5}{*}{$q_'+ str(it+1) + '$} &'
            elif a == 'nh' and query == 'query':
                delimiter = '\\parbox[t]{2mm}{\multirow{6}{*}{\\rotatebox{90}{query ' + str(it + 1) + '}}} &'

            if a == 'rrt_star_last':
                tab.writelines(delimiter + '\\footnotesize{' + names[a]  + '} & ' + '\multicolumn{1}{c}{-}' + ' & ' + '\multicolumn{1}{c|}{-}' + '& ' +
                               '$' + format(results[2], l_p) + '$ & $\pm' + format(results[3], l_p) + '$ &' +
                               '$' + format(results[4], h_p) + '$ & $\pm' + format(results[5], h_p) + '$ &' +
                               '$' + str(results[6]) + '\%$ \\\\' + '\n')
            else:
                tab.writelines(delimiter + '\\footnotesize{' + names[a]  + '} & $' + format(results[0], m_p) + '$ & $\pm' + format(results[1], m_p) + '$ & ' +
                               '$' + format(results[2], l_p) + '$ & $\pm' + format(results[3], l_p) + '$ &' +
                               '$' + format(results[4], h_p) + '$ & $\pm' + format(results[5], h_p) + '$ &' +
                               '$' + str(results[6]) + '\%$ \\\\' + '\n')

        tab.writelines("\\bottomrule"+ '\n')

    tab.writelines('\end{tabular}\end{small}\end{center}\caption{' + remap[m] + ' - bicycle}\label{tab:bi-' + latex[m] + '}')
    tab.writelines('\end{table}' + '\n')
    tab.close()


if __name__ == '__main__':
    for m in maps:
        parse_logs(m)
        #table(m)
