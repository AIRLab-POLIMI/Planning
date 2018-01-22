import os
import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import joypy

alg_name = 'nh'

labels = ['length', 'time', 'roughness', 'success rate']

df = pd.read_csv(env_name + "ancestor_fixed.csv")

params = {'axes.labelsize': 18,'axes.titlesize': 18, 'text.fontsize': 18, 'legend.fontsize': 18, 'xtick.labelsize': 18, 'ytick.labelsize': 18}
matplotlib.rcParams.update(params)

for l in labels :
    filtered_df = {}
    p_cols = []
    
    min_l = df.l.min()
    max_l = df.l.max()

    for a in range(min_l, max_l) :
        filtered_df[str(a)] = df_ra.loc[df['Agents'] == a]['Times']
        p_cols.append(str(a))


    #print("\n\n"+alg+"\n\n")
    #print(filtered_df)

    '''joypy.joyplot(filtered_df, column = p_cols, by=None, grid=True,
                xlabelsize=None, xrot=None, ylabelsize=None, yrot=None,
                ax=None, figsize=None,
                hist=False, bins=10,
                fade=False, ylim='max',
                fill=True, linecolor=None,
                overlap=1, background=None,
                labels=None, xlabels=True, ylabels=True,
                range_style='all',
                x_range=None,
                title=None,
                colormap=None,
                bw_method=0.3)'''


sns.boxplot(x = "Agents", y = "Times", hue = "Algorithm", data = df, showfliers = False)
plt.xlabel('Agents')
plt.ylabel('Solution Length')

#plt.xlabel('Times')
plt.tight_layout()

plt.savefig(env_name + '_lengths.pdf')
plt.show()
