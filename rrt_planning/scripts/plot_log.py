import os
import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

#maps = ['open', 'offices', 'map']
maps = ['open']
labels = ['length', 'time', 'roughness', 'success rate']
algorithms = ['theta_star_rrt', 'nh_s2', 'nh_s2_p1', 'nh_s3', 'nh_s3_p1',
              'forward_nh_s2', 'forward_nh_s2_p1', 'forward_nh_s3', 'forward_nh_s3_p1']


def plot(m):
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + m + '.csv')
    df_a = df[(df.algorithm == 'theta_star_rrt') | (df.algorithm == 'nh_s3')]

    params = {'axes.labelsize': 10,'axes.titlesize': 10, 'text.fontsize': 10, 'legend.fontsize': 10, 'xtick.labelsize': 10, 'ytick.labelsize': 10}
    matplotlib.rcParams.update(params)

    fig, axes = plt.subplots(3, 1)

    plt.xlabel('Algorithm')
    plt.ylabel('Time (s)')
    time = sns.boxplot(x = "conf", y = "time", hue='algorithm', data = df_a, showfliers = False, ax = axes[0])

    plt.xlabel('Algorithm')
    plt.ylabel('Length')
    length = sns.boxplot(x = "conf", y = "length", hue='algorithm', data = df_a, showfliers = False, ax = axes[1])

    plt.xlabel('Algorithm')
    plt.ylabel('Roughness')
    roughness = sns.boxplot(x = "conf", y = "roughness", hue='algorithm', data = df_a, showfliers = False, ax = axes[2])

    for ax in fig.axes:
        matplotlib.pyplot.sca(ax)
        ax.legend_.remove()
        ax.set_yscale('log')
        x0,x1 = ax.get_xlim()
        y0,y1 = ax.get_ylim()
        ax.set_aspect(abs(x1-x0)/(2*abs(y1-y0)))
        ax.xaxis.label.set_visible(False)
        ax.yaxis.label.set_visible(False)




    plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.25, wspace=0.35)
    plt.show()



    #length = sns.boxplot(x = "conf", y = "length", hue = "algorithm", data = df, showfliers = False)
    #plt.xlabel('Configuration')
    #plt.savefig(m + '_length.pdf')
    #plt.show()


if __name__ == '__main__':
    for m in maps:
        plot(m)
