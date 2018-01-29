import matplotlib.pyplot as plt
import os
import pandas as pd

cw = os.getcwd()

algorithms = ['nh']

theta = pd.read_csv(cw + '/theta_star_rrt_values.csv')

for alg in algorithms:
    df = pd.read_csv(cw + '/' + alg + '_corner_fixed.csv')
    corners = df['ancestors']
    
    plt.subplot(211)
    plt.title('samples on corner = 2')
    data = df['time']
    value = theta['time']
    values = [value, value, value, value, value, value]

    plt.ylim(ymin=0, ymax=max(data))
    if alg == 'nh':
        plt.plot(corners, data, 'red', label='nh')
    elif alg == 'forward_nh':
        plt.plot(corners, data, 'blue', label='forward_nh')
    plt.plot(corners, values, 'black', label='theta*RRT')
    plt.xlabel('ancestors')
    plt.ylabel('time (s)')
    
    plt.subplot(212)
    data = df['success_rate']
    value = theta['success_rate']
    values = [value, value, value, value, value, value]

    plt.ylim(ymin=0, ymax=100)
    plt.yticks(range(0, 100, 10))
    if alg == 'nh':
        plt.plot(corners, data, 'red', label='nh')
    elif alg == 'forward_nh':
        plt.plot(corners, data, 'blue', label='forward_nh')
    plt.plot(corners, values, 'black', label='theta*RRT')    
    plt.xlabel('ancestors')
    plt.ylabel('success rate (%)')
        
plt.legend(loc='lower right')
plt.show()
