import os
import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

#maps = ['open', 'offices', 'map']
maps = ['buildings']
labels = ['length', 'time', 'roughness', 'success rate']
algorithms = ['theta_star_rrt', 'nh']


def plot(m):
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + m + '.csv')
    df_a = df[(df.algorithm == 'theta_star_rrt') | (df.algorithm == 'nh')]

    params = {'axes.labelsize': 10,'axes.titlesize': 10, 'text.fontsize': 10, 'legend.fontsize': 10, 'xtick.labelsize': 10, 'ytick.labelsize': 10}
    matplotlib.rcParams.update(params)

    fig, axes = plt.subplots(3, 1)

    plt.xlabel('Algorithm')
    plt.ylabel('Time (s)')
    nh = df_a[df.algorithm == 'nh'].groupby('conf')
    sorted_time = pd.DataFrame({col:vals['time'] for col,vals in nh}).median().sort_values(ascending=True)
    time = sns.boxplot(x = "conf", y = "time", hue='algorithm', data = df_a, showfliers = False, order = sorted_time.index, ax = axes[0])

    plt.xlabel('Algorithm')
    plt.ylabel('Length')
    sorted_length = pd.DataFrame({col:vals['length'] for col,vals in nh}).median().sort_values(ascending=True)
    length = sns.boxplot(x = "conf", y = "length", hue='algorithm', data = df_a, showfliers = False, order = sorted_time.index, ax = axes[1])

    plt.xlabel('Algorithm')
    plt.ylabel('Roughness')
    sorted_roughness = pd.DataFrame({col:vals['roughness'] for col,vals in nh}).median().sort_values(ascending=True)
    roughness = sns.boxplot(x = "conf", y = "roughness", hue='algorithm', data = df_a, showfliers = False, order = sorted_time.index, ax = axes[2])

    for ax in fig.axes:
        matplotlib.pyplot.sca(ax)
        ax.legend_.remove()
        #ax.set_yscale('log')
        x0,x1 = ax.get_xlim()
        y0,y1 = ax.get_ylim()
        ax.set_aspect(abs(x1-x0)/(3*abs(y1-y0)))
        ax.xaxis.label.set_visible(False)
        ax.yaxis.label.set_visible(False)

    plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.25, wspace=0.35)
    plt.show()

def plot_lines(m):
    conf = (0, 1, 2, 3, 4, 5, 6, 7, 8, 9)
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + m + '.csv')
    df_a = df[(df.algorithm == 'theta_star_rrt') | (df.algorithm == 'nh')]

    params = {'axes.labelsize': 10,'axes.titlesize': 10, 'text.fontsize': 10, 'legend.fontsize': 10, 'xtick.labelsize': 10, 'ytick.labelsize': 10}
    matplotlib.rcParams.update(params)

    fig, axes = plt.subplots(4, 1)
    data = df_a[df.algorithm == 'nh'].groupby('conf')
    
    ##Length
    nh_l = pd.DataFrame({col:vals['length'] for col,vals in df_a[df.algorithm == 'nh'].groupby('conf')}).median().sort_values(ascending=True)
    theta_l = pd.DataFrame({col:vals['length'] for col,vals in df_a[df.algorithm == 'theta_star_rrt'].groupby('conf')}).median().values
    index = nh_l.index
    
    c = theta_l.copy()
    for i in range(len(index)):
        theta_l[i] = c[index[i]]

    axes[0].plot(nh_l.values, label='nh', color='green', marker='o')
    axes[0].plot(theta_l, label='theta_star_rrt', color='red', marker='o')
    axes[0].set_xticklabels(index)
    axes[0].legend(bbox_to_anchor=(1.04,1), loc='upper left')
    axes[0].set_title("Length (m)")
    
    ##Time
    nh_t = pd.DataFrame({col:vals['time'] for col,vals in df_a[df.algorithm == 'nh'].groupby('conf')}).median().values
    theta_t = pd.DataFrame({col:vals['time'] for col,vals in df_a[df.algorithm == 'theta_star_rrt'].groupby('conf')}).median().values
    

    c = theta_t.copy()
    c2 = nh_t.copy()
    for i in range(len(index)):
        theta_t[i] = c[index[i]]
        nh_t[i] = c2[index[i]]

    axes[1].plot(nh_t, label='nh', color='green', marker='o')
    axes[1].plot(theta_t, label='theta_star_rrt', color='red', marker='o')
    axes[1].set_xticklabels(index)
    #axes[1].legend(loc='upper left')
    axes[1].set_title("Time (s)")
    

    #Roughness
    nh_r = pd.DataFrame({col:vals['roughness'] for col,vals in df_a[df.algorithm == 'nh'].groupby('conf')}).median().sort_values(ascending=True)
    theta_r = pd.DataFrame({col:vals['roughness'] for col,vals in df_a[df.algorithm == 'theta_star_rrt'].groupby('conf')}).median().values

    c = theta_r.copy()
    c2 = nh_r.copy()
    for i in range(len(index)):
        theta_r[i] = c[index[i]]
        nh_r[i] = c2[index[i]]

    axes[2].plot(nh_r.values, label='nh', color='green', marker='o')
    axes[2].plot(theta_r, label='theta_star_rrt', color='red', marker='o')
    axes[2].set_xticklabels(index)
    #axes[2].legend(loc='upper left')
    axes[2].set_title("Roughness")

    #Success rate
    conf = max(df_a.conf) + 1
    for alg in algorithms:
        rate = np.arange(conf)
        for c in range(0, conf):
            d = df_a[(df_a.algorithm == alg) & (df_a.conf == c)]
            success = d[d.length!=0]
            rate[c] = float(success.shape[0]) / float(d.shape[0]) * 100
        c = rate.copy()
        for i in range(len(index)):
            rate[i] = c[index[i]]            
        if alg == 'nh':
            axes[3].plot(rate, label='nh', color='green', marker='o')
        elif alg == 'theta_star_rrt':
            axes[3].plot(rate, label='theta_star_rrt', color='red', marker='o')
    axes[3].set_title("Success rate %")
    axes[3].set_xticklabels(index)
    plt.ylim(ymin=0, ymax=110)


    for ax in fig.axes:
        matplotlib.pyplot.sca(ax)
        #ax.set_xticks(nh.index)
        #ax.legend_.remove()
        #ax.set_yscale('log')
        x0,x1 = ax.get_xlim()
        y0,y1 = ax.get_ylim()
        ax.set_aspect(abs(x1-x0)/(3*abs(y1-y0)))
        ax.xaxis.label.set_visible(False)
        ax.yaxis.label.set_visible(False)

    plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.25, wspace=0.35)
    #plt.show()
    fig.tight_layout()
    plt.savefig(m + "_graph.pdf")



if __name__ == '__main__':
    for m in maps:
        #plot(m)
        plot_lines(m)
