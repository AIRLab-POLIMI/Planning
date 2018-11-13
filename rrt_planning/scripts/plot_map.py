import os
import sys
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import pandas as pd
import seaborn as sns


#compare everything
#algorithms = ['nh', 'nh_l2', 'rrt', 'rrt_star_first', 'rrt_star_last', 'theta_star_rrt', 'voronoi_rrt']
#maps = ['open', 'map', 'buildings', 'offices']


#compare just some
#algorithms = ['rrt_star_last', 'rrt']
algorithms =['nh', 'rrt', 'rrt_star_last', 'theta_star_rrt', 'voronoi_rrt']
#maps = ['buildings']
maps = ['open', 'map', 'offices', 'buildings']

def plot(m):
    wd = os.getcwd()

    magics = PdfPages(wd + '/results/' + m + '.pdf')
    #magics = PdfPages(wd + '/results/' + m + '_fast.pdf')
    params = {'axes.labelsize': 10,'axes.titlesize': 15, 'text.fontsize': 10, 'legend.fontsize': 10, 'xtick.labelsize': 10, 'ytick.labelsize': 10}
    matplotlib.rcParams.update(params)

    f = pd.read_csv(wd + '/results/compare/' + m + '.csv')
    #f_in = f[(f.algorithm == 'rrt_star_last') | (f.algorithm == 'rrt_star_first')]
    f_in = f
    #f_in = f[(f.algorithm == 'nh') | (f.algorithm == 'nh_l2') | (f.algorithm == 'theta_star_rrt') | (f.algorithm == 'voronoi_rrt')]
    conf = max(f_in.conf) + 1

    for i in range(0,1):
        #df = f_in[f_in.conf == i]
        df = f_in
        #df = df[df.algorithm != 'rrt_star_first']
        #df_time = df[(df.algorithm == 'rrt_star_first') | (df.algorithm == 'rrt')]
        plt.clf()
        fig, axes = plt.subplots(1, 3, gridspec_kw = {'width_ratios':[1, 1, 1, 1]}, figsize=(20,7))

        #df_time = df_time[df_time.length!=0]
        df_time = df[(df.algorithm == 'rrt_star_last') | (df.algorithm == 'rrt')]
        df = df[df.length != 0]
        df_time = df[df.algorithm != 'rrt_star_last']
        #df_success = df_success[df_success.length != 0]
        #df_time = df_success[(df_success.algorithm != 'rrt_star_last') & (df_success.algorithm != 'rrt_star_first') ]

        time = sns.boxplot(x = "algorithm", y = "time", data = df_time, showfliers = False, ax = axes[0])
        length = sns.boxplot(x = "algorithm", y = "length", data = df, showfliers = False, ax = axes[1])
        roughness = sns.boxplot(x = "algorithm", y = "roughness", data = df, showfliers = False, ax = axes[2])
        names = ['RRT', 'RRT*']


        x = np.arange(len(algorithms))
        rate = np.arange(len(algorithms))

        titles = ['Time (s)', 'Length (m)', 'Smoothness', 'Success rate (%)']
        for ax in fig.axes:
            matplotlib.pyplot.sca(ax)
            plt.xticks(rotation=90)
            x0,x1 = ax.get_xlim()
            y0,y1 = ax.get_ylim()
            ax.set_aspect(abs(x1-x0)/abs(y1-y0))
            ax.xaxis.label.set_visible(False)
            ax.yaxis.label.set_visible(False)
            ax.set_title(titles[fig.axes.index(ax)])
            #ax.set_xticklabels(names)

        fig.tight_layout()
        plt.subplots_adjust(top=0.90, bottom=0.1, left=0.05, right=0.95, hspace=0.25, wspace=0.35)
        fig.suptitle(m + ' ' + str(i), fontsize=20)
        magics.savefig()
        plt.close()

    magics.close()


if __name__ == '__main__':
    for m in maps:
        plot(m)
