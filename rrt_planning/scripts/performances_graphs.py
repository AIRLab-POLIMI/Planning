import os
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

import warnings
from matplotlib.cbook import MatplotlibDeprecationWarning
warnings.simplefilter(action='ignore', category=FutureWarning)
warnings.simplefilter(action='ignore', category=MatplotlibDeprecationWarning)

path = '/home/dave/Planning/plots/'
params = {'axes.labelsize': 15,'axes.titlesize': 15, #'text.fontsize': 15,
          'legend.fontsize': 20, 'xtick.labelsize': 20, 'ytick.labelsize': 15}

models = ['differential_drive', 'bicycle']
queries = [r'$q_1$', r'$q_2$', r'$q_3$', r'$q_4$', r'$q_5$']
maps = ['floor', 'offices', 'open', 'street']

algorithms =['WTS', 'T*-RRT', 'V-RRT', 'RRT', 'RRT*', 'RRT*-first']

palette = {'WTS': '#ff7f7f', 'T*-RRT': '#ffb481', 'V-RRT': '#9cff86',
           'RRT': '#89d8ff', 'RRT*-first':'#efa7de', 'RRT*': '#9278dd'}

names = {'offices': 'Offices', 'street': 'Street',
             'open': 'Open', 'floor': 'Floor'}

params = {'axes.labelsize': 10,'axes.titlesize': 15, #'text.fontsize': 10,
          'legend.fontsize': 8, 'xtick.labelsize': 10, 'ytick.labelsize': 8}


def get_ordering(m):
    if m == 'floor':
        order_index = [0,2,1,3,4]
    elif m == 'offices':
        order_index = [1,4,2,0,3]
    elif m == 'open':
        order_index = [0,4,2,3,1]
    elif m == 'street':
        order_index = [2,1,0,4,3]
    return order_index


def time_only(k, m, savefig=False, **fig_args):
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')
    order_index = get_ordering(m)

    matplotlib.rcParams.update(params)

    fig = plt.figure(**fig_args)
    df = df[df.length != 0]
    df_time = df[(df.algorithm != 'RRT*') & (df.algorithm != 'RRT*-first')
                 & (df.algorithm != 'RRT')]

    plt.title('Time (s)', fontsize=20)
    ax = sns.pointplot(x="conf", y="time", hue='algorithm', palette=palette,
                       data=df_time, capsize=.1, order=order_index)
    ax.ticklabel_format(axis='y', style='sci', scilimits=(0,1))
    plt.setp(ax.get_legend().get_texts(), fontsize='15')
    ax.yaxis.get_offset_text().set_fontsize(12)
    ax.get_legend().set_title('')

    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    ax.xaxis.label.set_visible(False)
    ax.yaxis.label.set_visible(False)
    ax.set_xticklabels(queries)

    fig.tight_layout()
    
    if savefig:
        plt.savefig(path + k + '/' + m + '/time.pdf',
                    bbox_inches='tight', pad_inches=0)


def lenght_only(k, m, savefig=False, **fig_args):
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')
    order_index = get_ordering(m)

    matplotlib.rcParams.update(params)

    fig = plt.figure(**fig_args)
    df = df[df.length != 0]
    df_lenght = df[(df.algorithm != 'RRT*-first')]

    plt.title('Length (m)', fontsize=20)
    ax = sns.pointplot(x="conf", y="length", hue='algorithm', palette=palette,
                       data=df_lenght, capsize=.1, order=order_index)
    ax.ticklabel_format(axis='y', style='sci', scilimits=(0,1))
    plt.setp(ax.get_legend().get_texts(), fontsize='15')
    ax.yaxis.get_offset_text().set_fontsize(12)
    ax.get_legend().set_title('')

    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    ax.xaxis.label.set_visible(False)
    ax.yaxis.label.set_visible(False)
    ax.set_xticklabels(queries)

    fig.tight_layout()
    
    if savefig:
        plt.savefig(path + k + '/' + m + '/lenght.pdf',
                    bbox_inches='tight', pad_inches=0)


def kappa(k, m, savefig=False, **fig_args):
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')

    df = df[df.algorithm != 'RRT*-first']
    df = df[df.length != 0]

    fig = plt.figure(**fig_args)

    matplotlib.rcParams.update(params)
    plt.title(r'Smoothness (rad$^2$)', fontsize=20)
    ax = sns.boxplot(x="algorithm", y="roughness", data=df, showfliers=False,
                     palette=palette)
    ax.ticklabel_format(axis='y', style='sci', scilimits=(0,1))
    ax.yaxis.get_offset_text().set_fontsize(12)

    labels = ['WTS', 'T*-RRT', 'V-RRT', 'RRT', 'RRT*']
    index = np.arange(5)
    plt.xticks(index, labels, fontsize=15)
    plt.yticks(fontsize=15)

    ax.xaxis.label.set_visible(False)
    ax.yaxis.label.set_visible(False)

    fig.tight_layout()

    if savefig:
        plt.savefig(path + k + '/' + m + '/smoothness.pdf',
                    bbox_inches='tight', pad_inches=0)


def success_rate(k, m, savefig=False, **fig_args):
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')

    order_index = get_ordering(m)

    matplotlib.rcParams.update(params)
    algorithms = ['WTS', 'T*-RRT', 'V-RRT', 'RRT', 'RRT*']
    bar_width = 0.15
    index = np.arange(5)
    rate = np.arange(5)
    fig = plt.figure(**fig_args)
    for alg in algorithms:
        rate = [0, 0, 0, 0, 0]
        for c in range(0, 5):
            d = df[(df.algorithm == alg) & (df.conf == order_index[c])]
            success = d[d.length != 0]
            rate[c] = float(success.shape[0]) / float(d.shape[0]) * 100
            if alg == 'WTS':
                rate[c] = 100
        plt.bar(index + algorithms.index(alg) * bar_width, rate, bar_width,
                label=alg, color=palette[alg])

    plt.xticks(index + 2.5 * bar_width, queries)
    plt.ylim(ymax=110)
    plt.legend(bbox_to_anchor=(1.0, 1), loc='upper right')
    plt.title(names[m])

    if savefig:
        plt.savefig(path + k + '/' + m + '/success_rate.pdf',
                    bbox_inches='tight', pad_inches=0)


def rrt_time(k, m, savefig=False, **fig_args):
    wd = os.getcwd()

    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')

    matplotlib.rcParams.update(params)
    plt.figure(**fig_args)

    df = df[df.length != 0]
    df = df[(df.algorithm == 'RRT*-first') | (df.algorithm == 'RRT')]

    plt.title('Time (s)')
    sns.boxplot(x="conf", y="time", hue='algorithm', palette=palette,
                showfliers=False, data=df)
    index = np.arange(5)
    plt.xticks(index, queries)

    if savefig:
        plt.savefig(path + k + '/' + m + '/rrt_time.pdf',
                    bbox_inches='tight', pad_inches=0)


def time_length(k, m, savefig=False, **fig_args):
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')
    order_index = get_ordering(m)

    matplotlib.rcParams.update(params)

    fig, axes = plt.subplots(3, 1, **fig_args)

    df = df[df.length != 0]
    df_length_long = df[
        (df.algorithm == 'RRT*') | (df.algorithm == 'RRT*-first')
        | (df.algorithm == 'RRT')]
    df = df[df.algorithm != 'RRT*-first']
    df_time = df[(df.algorithm != 'RRT*') & (df.algorithm != 'RRT*-first')
                 & (df.algorithm != 'RRT')]

    axes[0].set_title('Time (s)', fontsize=12)
    sns.pointplot(x="conf", y="time", hue='algorithm', palette=palette,
                  data=df_time, capsize=.1, order=order_index, ax=axes[0])

    axes[1].set_title('Length (m)', fontsize=12)
    sns.pointplot(x="conf", y="length", hue='algorithm', palette=palette,
                  data=df_time, capsize=.1, order=order_index, ax=axes[1])
    axes[1].legend(loc='upper left')

    axes[2].set_title('Length (m)', fontsize=12)
    sns.pointplot(x="conf", y="length", hue='algorithm', palette=palette,
                  data=df_length_long, capsize=.1, order=order_index,
                  ax=axes[2])
    axes[2].legend(loc='upper left')

    for ax in fig.axes:
        matplotlib.pyplot.sca(ax)
        ax.xaxis.label.set_visible(False)
        ax.yaxis.label.set_visible(False)
        ax.set_xticklabels(queries)

    fig.tight_layout()
    plt.subplots_adjust(top=0.90, bottom=0.1, left=0.05, right=0.95,
                        hspace=0.25, wspace=0.15)

    if savefig:
        plt.savefig(path + k + '/' + m + '/time_lenght.pdf')


def plot_all(k, m, savefig=False, **fig_args):
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')

    df = df[df.length != 0]
    order_index = get_ordering(m)

    matplotlib.rcParams.update(params)

    fig, axes = plt.subplots(2, 2, **fig_args)

    df = df[df.length != 0]
    
    df_short = df[(df.algorithm != 'RRT*') & (df.algorithm != 'RRT*-first')
               & (df.algorithm != 'RRT')]
    df_long = df[(df.algorithm == 'RRT*') | (df.algorithm == 'RRT*-first')
               | (df.algorithm == 'RRT')]

    axes[0][0].set_title('time (s)',fontsize=12)
    sns.pointplot(x="conf", y="time", hue='algorithm', palette=palette,
                  data=df_short, capsize=.1, order=order_index, ax=axes[0][0])

    axes[0][1].set_title('length (m)',fontsize=12)
    sns.pointplot(x="conf", y="length", hue='algorithm', palette=palette,
                  data=df_short, capsize=.1, order=order_index, ax=axes[0][1])

    sns.pointplot(x="conf", y="length", hue='algorithm', palette=palette,
                  data=df_long, capsize=.1, order=order_index, ax=axes[1][1])

    axes[1][1].set_title('length (m)' ,fontsize=12)
    
    algorithms =['WTS', 'T*-RRT', 'V-RRT', 'RRT', 'RRT*-first', 'RRT*']
    sns.boxplot(x="algorithm", y="roughness", data=df, showfliers=False,
                palette=palette, ax=axes[1][0])

    for ax in fig.axes:
        matplotlib.pyplot.sca(ax)
        ax.xaxis.label.set_visible(False)
        ax.yaxis.label.set_visible(False)
        if ax != axes[1][0]:
            ax.legend(loc='upper left')
        ax.set_xticklabels(queries)

    axes[1][0].set_xticklabels(algorithms, fontsize=9)
    axes[1][0].set_title('smoothness(rad)',fontsize=12)

    fig.tight_layout()
    plt.subplots_adjust(top=0.90, bottom=0.1, left=0.05, right=0.95,
                        hspace=0.25, wspace=0.15)
    
    if savefig:
        plt.savefig(path + k + '/' + m + '/all.pdf',
                    bbox_inches='tight', pad_inches=0)


if __name__ == '__main__':
    show = False
    savefig = True
    for k in models:
        print('# ', k)
        for m in maps:
            print('   - ', m)

            print('      . all')
            plot_all(k, m, savefig, num=1)
            
            print('      . time_only')
            time_only(k, m, savefig, num=2, figsize=(5., 3.))
            
            print('      . lenght_only')
            lenght_only(k, m, savefig, num=3, figsize=(5., 3.))
            
            print('      . kappa')
            kappa(k, m, savefig, num=4, figsize=(5., 3.))
            
            print('      . success_rate')
            success_rate(k, m, savefig, num=5, figsize=(5., 4.))
            
            print('      . rrt_time')
            rrt_time(k, m, savefig, num=6, figsize=(5., 4.))
            
            print('      . time_length')
            time_length(k, m, savefig, num=7, figsize=(5., 12.))
            
            if show:
                plt.show()
            plt.close('all')
