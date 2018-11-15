def fuck_this_shit(m, k):  # TODO
    params = {'axes.labelsize': 10, 'axes.titlesize': 15, 'text.fontsize': 15,
              'legend.fontsize': 10, 'xtick.labelsize': 10,
              'ytick.labelsize': 10}

    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')

    order_index = get_ordering(m)

    matplotlib.rcParams.update(params)

    fig, axes = plt.subplots(1, 2)
    df = df[df.length != 0]
    # df = df[df.algorithm != 'RRT*']
    df_short = df[(df.algorithm != 'RRT*') & (df.algorithm != 'RRT*-first')
                  & (df.algorithm != 'RRT')]
    df_long = df[(df.algorithm != 'RRT*') & (df.algorithm != 'RRT*-first')]

    axes[0].set_title('time (s)')
    time = sns.pointplot(x="conf", y="time", hue='algorithm', palette=palette,
                         data=df_short,
                         capsize=.1, order=order_index, ax=axes[1])
    plt.close()
    axes[1].set_ylim(ymax=12)

    length = sns.pointplot(x="conf", y="time", hue='algorithm', palette=palette,
                           data=df_long,
                           capsize=.1, order=order_index, ax=axes[0])
    axes[0].set_ylim(ymin=-5)
    plt.close()

    for ax in fig.axes:
        matplotlib.pyplot.sca(ax)
        x0, x1 = ax.get_xlim()
        y0, y1 = ax.get_ylim()
        ax.set_aspect(abs(x1 - x0) / (2 * abs(y1 - y0)))
        ax.xaxis.label.set_visible(False)
        ax.yaxis.label.set_visible(False)
        ax.legend(loc='upper left')
        # ax.legend(bbox_to_anchor=(1.04,1), loc='upper left')
        ax.set_xticklabels(queries)

        # ax.set_xticks(fontsize=15)

    fig.tight_layout()
    plt.subplots_adjust(top=0.90, bottom=0.1, left=0.05, right=0.95,
                        hspace=0.25, wspace=0.15)
    plt.savefig(path + 'shenanings.pdf')  # TODO
    plt.show()


def one_length(m, k):  # TODO
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')

    order_index = get_ordering(m)

    df = df[df.length != 0]

    plt.clf()
    matplotlib.rcParams.update(params)
    sns.boxplot(x="algorithm", y="roughness", data=df, showfliers=False,
                palette=palette)

    index = np.arange(5)
    plt.xticks(index, queries)
    plt.xlabel('')
    plt.ylabel('')
    plt.title('length (m)')
    x0, x1 = plt.xlim()
    y0, y1 = plt.ylim()
    # ax.set_aspect(abs(x1-x0)/(2*abs(y1-y0)))
    plt.axes().set_aspect(18)

    # plt.show()

    plt.savefig(path + k + '/' + m + '/smoothness.pdf')


def box(m, k, c):  # TODO
    wd = os.getcwd()
    df = pd.read_csv(wd + '/results/compare/' + k + '/' + m + '.csv')

    order_index = get_ordering(m)

    # df = df[df.algorithm != 'RRT*-first']
    # df = df[df.algorithm != 'RRT*']
    df = df[df.length != 0]
    df = df[df.conf == int(c)]

    plt.clf()
    matplotlib.rcParams.update(params)
    sns.boxplot(x="algorithm", y="length", data=df, showfliers=False,
                palette=palette)
    labels = ['WTS', 'T*-RRT', 'V-RRT', 'RRT', "RRT*-first", "RRT*"]
    index = np.arange(5)
    plt.xticks(index, labels)
    plt.xlabel('')
    plt.ylabel('')
    plt.title('length')
    # plt.axes().set_aspect(18)

    plt.show()

    # plt.savefig(path + k + '/' + m + '/smoothness.pdf')


def random():  # TODO
    dd_rate = [100, 100, 100, 91, 60]
    bi_rate = [100, 100, 100, 80, 47]

    matplotlib.rcParams.update(params)
    algorithms = ['WTS', 'T*-RRT', 'V-RRT', 'RRT', 'RRT*']
    label = ['diffential drive', 'bicycle']

    bar_width = 0.10
    index = np.arange(2)
    rate = np.arange(2)
    plt.clf()
    for alg in algorithms:
        rate = [0, 0]
        rate[0] = dd_rate[algorithms.index(alg)]
        rate[1] = bi_rate[algorithms.index(alg)]
        plt.bar(index + algorithms.index(alg) * bar_width, rate, bar_width,
                label=alg, color=palette[alg])

    plt.xticks(index + 2.5 * bar_width, label, fontsize=15)
    plt.yticks(fontsize=15)
    plt.ylim(ymax=110)
    # plt.axes().set_aspect(0.010)
    # plt.title('Success rate')
    plt.legend(bbox_to_anchor=(1.2, 1), loc='upper right', fontsize=20)
    # plt.savefig(path + 'rate.pdf')
    plt.show()