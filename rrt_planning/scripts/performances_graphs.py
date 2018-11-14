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
queries = ['q1', 'q2', 'q3', 'q4', 'q5']
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


def time_length(k, m, n, savefig=False):
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')
    order_index = get_ordering(m)
    
    matplotlib.rcParams.update(params)

    fig, axes = plt.subplots(3, 1, num=n)

    df = df[df.length != 0]
    df_length_long = df[(df.algorithm == 'RRT*') | (df.algorithm == 'RRT*-first')
                     | (df.algorithm == 'RRT')]
    df = df[df.algorithm != 'RRT*-first']
    df_time = df[(df.algorithm != 'RRT*') & (df.algorithm != 'RRT*-first')
               & (df.algorithm != 'RRT')]

    axes[0].set_title('Time (s)', fontsize=12)
    time = sns.pointplot(x = "conf", y = "time", hue='algorithm', palette=palette, data = df_time,
                          capsize=.1, order = order_index, ax = axes[0])

    axes[1].set_title('Length (m)', fontsize=12)
    length = sns.pointplot(x = "conf", y = "length", hue='algorithm', palette=palette, data = df_time,
                          capsize=.1, order = order_index, ax = axes[1])
    axes[1].legend(loc='upper left')

    axes[2].set_title('Length (m)', fontsize=12)
    length = sns.pointplot(x = "conf", y = "length", hue='algorithm', palette=palette, data = df_length_long,
                          capsize=.1, order = order_index, ax = axes[2])
    axes[2].legend(loc='upper left')

    for ax in fig.axes:
        matplotlib.pyplot.sca(ax)
        x0,x1 = ax.get_xlim()
        y0,y1 = ax.get_ylim()
        ax.set_aspect(abs(x1-x0)/(3*abs(y1-y0)))
        ax.xaxis.label.set_visible(False)
        ax.yaxis.label.set_visible(False)
        ax.set_xticklabels(queries)

    fig.tight_layout()
    plt.subplots_adjust(top=0.90, bottom=0.1, left=0.05, right=0.95, hspace=0.25, wspace=0.15)
    
    if savefig:
        plt.savefig(path + k + '/' + m + '/time_lenght.pdf')

def time_only(k, m, n, savefig=False):
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')
    order_index = get_ordering(m)
    

    matplotlib.rcParams.update(params)

    fig = plt.figure(n)
    df = df[df.length != 0]
    df_time = df[(df.algorithm != 'RRT*') & (df.algorithm != 'RRT*-first') & (df.algorithm != 'RRT')]

    plt.title('Time (s)', fontsize=12)
    sns.pointplot(x = "conf", y = "time", hue='algorithm', palette=palette, data = df_time,
                          capsize=.1, order = order_index)

    for ax in fig.axes:
        ax.xaxis.label.set_visible(False)
        ax.yaxis.label.set_visible(False)
        ax.set_xticklabels(queries)

    fig.tight_layout()
    
    if savefig:
        plt.savefig(path + k + '/' + m + '/time.pdf')

def lenght_only(k, m, n, savefig=False):
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')
    order_index = get_ordering(m)
    

    matplotlib.rcParams.update(params)

    fig = plt.figure(n)
    df = df[df.length != 0]
    df_lenght = df[(df.algorithm != 'RRT*-first')]

    plt.title('Length (m)', fontsize=12)
    sns.pointplot(x = "conf", y = "length", hue='algorithm', palette=palette, data = df_lenght,
                  capsize=.1, order = order_index)

    for ax in fig.axes:
        ax.xaxis.label.set_visible(False)
        ax.yaxis.label.set_visible(False)
        ax.set_xticklabels(queries)

    fig.tight_layout()
    
    if savefig:
        plt.savefig(path + k + '/' + m + '/lenght.pdf')

def rrt_time(k, m, n, savefig=False):
    wd = os.getcwd()

    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')

    order_index = get_ordering(m)

    matplotlib.rcParams.update(params)
    fig = plt.figure(n)

    df = df[df.length != 0]
    df = df[(df.algorithm == 'RRT*-first') | (df.algorithm == 'RRT')]

    plt.title('Time (s)')
    time = sns.boxplot(x = "conf", y = "time", hue='algorithm', palette=palette, showfliers = False, data = df)
    index = np.arange(5)
    plt.xticks(index, queries)
    plt.axes().set_aspect(0.005)
    
    if savefig:
        plt.savefig(path + k + '/' + m + '/rrt_time.pdf')


def plot_all(k, m, n, savefig=False):
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')

    df = df[df.length != 0]
    order_index = get_ordering(m)

    matplotlib.rcParams.update(params)

    fig, axes = plt.subplots(2, 2, num=n)

    df = df[df.length != 0]
    
    df_short = df[(df.algorithm != 'RRT*') & (df.algorithm != 'RRT*-first')
               & (df.algorithm != 'RRT')]
    df_long = df[(df.algorithm == 'RRT*') | (df.algorithm == 'RRT*-first')
               | (df.algorithm == 'RRT')]

    axes[0][0].set_title('time (s)',fontsize=12)
    time = sns.pointplot(x = "conf", y = "time", hue='algorithm', palette=palette, data = df_short,
                          capsize=.1, order = order_index, ax = axes[0][0])

    axes[0][1].set_title('length (m)',fontsize=12)
    length = sns.pointplot(x = "conf", y = "length", hue='algorithm', palette=palette, data = df_short,
                          capsize=.1, order = order_index, ax = axes[0][1])

    length = sns.pointplot(x = "conf", y = "length", hue='algorithm', palette=palette, data = df_long,
                          capsize=.1, order = order_index, ax = axes[1][1])

    axes[1][1].set_title('length (m)' ,fontsize=12)
    
    algorithms =['WTS', 'T*-RRT', 'V-RRT', 'RRT', 'RRT*-first', 'RRT*']
    sns.boxplot(x = "algorithm", y = "roughness", data = df, showfliers = False, palette=palette, ax=axes[1][0])


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
    plt.subplots_adjust(top=0.90, bottom=0.1, left=0.05, right=0.95, hspace=0.25, wspace=0.15)
    
    if savefig:
        plt.savefig(path + k + '/' + m + '/all.pdf')

def success_rate(k, m, n, savefig=False):
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')

    order_index = get_ordering(m)

    matplotlib.rcParams.update(params)
    algorithms =['WTS', 'T*-RRT', 'V-RRT', 'RRT', 'RRT*']
    bar_width = 0.15
    index = np.arange(5)
    rate = np.arange(5)
    fig = plt.figure(n)
    for alg in algorithms:
        rate = [0,0,0,0,0]
        for c in range(0, 5):
            d = df[(df.algorithm == alg) & (df.conf == order_index[c])]
            success = d[d.length!=0]
            rate[c] = float(success.shape[0]) / float(d.shape[0]) * 100
            if alg == 'WTS':
                rate[c] = 100
        plt.bar(index + algorithms.index(alg)*bar_width, rate, bar_width, label=alg, color=palette[alg])

    plt.xticks(index + 2.5*bar_width, queries)
    plt.ylim(ymax = 110)
    plt.legend(bbox_to_anchor=(1.0,1), loc='upper right')
    plt.title(names[m])
    
    if savefig:
        plt.savefig(path + k + '/' + m + '/success_rate.pdf')

def kappa(k, m, n, savefig=False):
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')
    order_index = get_ordering(m)

    df = df[df.algorithm != 'RRT*-first']
    df = df[df.length != 0]

    fig = plt.figure(n)
    
    matplotlib.rcParams.update(params)
    sns.boxplot(x = "algorithm", y = "roughness", data = df, showfliers = False, palette=palette)
    labels = ['WTS', 'T*-RRT', 'V-RRT', 'RRT', 'RRT*']
    index = np.arange(5)
    plt.xticks(index, labels, fontsize=15)
    plt.yticks(fontsize=13)
    plt.xlabel('')
    plt.ylabel('rad^2', fontsize=10)

    if savefig:
        plt.savefig(path + k + '/' + m + '/smoothness.pdf')
        
        
def fuck_this_shit(m,k): #TODO
    params = {'axes.labelsize': 10,'axes.titlesize': 15, 'text.fontsize': 15,
              'legend.fontsize': 10, 'xtick.labelsize': 10, 'ytick.labelsize': 10}

    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')

    order_index = get_ordering(m)

    matplotlib.rcParams.update(params)

    fig, axes = plt.subplots(1, 2)
    df = df[df.length != 0]
    #df = df[df.algorithm != 'RRT*']
    df_short = df[(df.algorithm != 'RRT*') & (df.algorithm != 'RRT*-first')
                  & (df.algorithm != 'RRT')]
    df_long = df[ (df.algorithm != 'RRT*') & (df.algorithm != 'RRT*-first')]

    axes[0].set_title('time (s)')
    time = sns.pointplot(x = "conf", y = "time", hue='algorithm', palette=palette, data = df_short,
                          capsize=.1, order = order_index, ax = axes[1])
    plt.close()
    axes[1].set_ylim(ymax=12)

    length = sns.pointplot(x = "conf", y = "time", hue='algorithm', palette=palette, data = df_long,
                          capsize=.1, order = order_index, ax = axes[0])
    axes[0].set_ylim(ymin=-5)
    plt.close()

    for ax in fig.axes:
        matplotlib.pyplot.sca(ax)
        x0,x1 = ax.get_xlim()
        y0,y1 = ax.get_ylim()
        ax.set_aspect(abs(x1-x0)/(2*abs(y1-y0)))
        ax.xaxis.label.set_visible(False)
        ax.yaxis.label.set_visible(False)
        ax.legend(loc='upper left')
        #ax.legend(bbox_to_anchor=(1.04,1), loc='upper left')
        ax.set_xticklabels(queries)

        #ax.set_xticks(fontsize=15)




    fig.tight_layout()
    plt.subplots_adjust(top=0.90, bottom=0.1, left=0.05, right=0.95, hspace=0.25, wspace=0.15)
    plt.savefig(path +'shenanings.pdf') #TODO
    plt.show()

def one_length(m, k): #TODO
     wd = os.getcwd()
     df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')
     
     order_index = get_ordering(m)

     df = df[df.length != 0]

     plt.clf()
     matplotlib.rcParams.update(params)
     sns.boxplot(x = "algorithm", y = "roughness", data = df, showfliers = False, palette=palette)

     index = np.arange(5)
     plt.xticks(index, queries)
     plt.xlabel('')
     plt.ylabel('')
     plt.title('length (m)')
     x0,x1 = plt.xlim()
     y0,y1 = plt.ylim()
        #ax.set_aspect(abs(x1-x0)/(2*abs(y1-y0)))
     plt.axes().set_aspect(18)

     #plt.show()

     plt.savefig(path + k + '/' + m + '/smoothness.pdf')

def box(m, k, c): #TODO
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')

    order_index = get_ordering(m)

    #df = df[df.algorithm != 'RRT*-first']
    #df = df[df.algorithm != 'RRT*']
    df = df[df.length != 0]
    df = df[df.conf == int(c)]

    plt.clf()
    matplotlib.rcParams.update(params)
    sns.boxplot(x = "algorithm", y = "length", data = df, showfliers = False, palette=palette)
    labels = ['WTS', 'T*-RRT', 'V-RRT', 'RRT', "RRT*-first", "RRT*"]
    index = np.arange(5)
    plt.xticks(index, labels)
    plt.xlabel('')
    plt.ylabel('')
    plt.title('length')
    #plt.axes().set_aspect(18)

    plt.show()

    #plt.savefig(path + k + '/' + m + '/smoothness.pdf')

def random(): #TODO
    dd_rate = [100, 100, 100, 91, 60]
    bi_rate = [100, 100, 100, 80, 47]

    matplotlib.rcParams.update(params)
    algorithms =['WTS', 'T*-RRT', 'V-RRT', 'RRT', 'RRT*']
    label = ['diffential drive', 'bicycle']

    bar_width = 0.10
    index = np.arange(2)
    rate = np.arange(2)
    plt.clf()
    for alg in algorithms:
        rate = [0,0]
        rate[0] = dd_rate[algorithms.index(alg)]
        rate[1] = bi_rate[algorithms.index(alg)]
        plt.bar(index + algorithms.index(alg)*bar_width, rate, bar_width, label=alg, color=palette[alg])

    plt.xticks(index + 2.5*bar_width, label, fontsize=15)
    plt.yticks(fontsize=15)
    plt.ylim(ymax = 110)
    #plt.axes().set_aspect(0.010)
    #plt.title('Success rate')
    plt.legend(bbox_to_anchor=(1.2,1), loc='upper right', fontsize=20)
    #plt.savefig(path + 'rate.pdf')
    plt.show()


if __name__ == '__main__':
    show = False
    savefig = True
    for k in models:
        print('# ', k)
        for m in maps:
            print('   - ', m)

            print('      . all')
            plot_all(k, m, 1, savefig)
            
            print('      . time_only')
            time_only(k, m, 2, savefig)
            
            print('      . lenght_only')
            lenght_only(k, m, 3, savefig)
            
            print('      . kappa')
            kappa(k, m, 4, savefig)
            
            print('      . success_rate')
            success_rate(k, m, 5, savefig)
            
            print('      . rrt_time')
            rrt_time(k, m, 6, savefig)
            
            print('      . time_length')
            time_length(k, m, 7, savefig)
            
            if show:
                plt.show()
            plt.close('all')
