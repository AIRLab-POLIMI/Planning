import os
import sys
import pandas as pd

if __name__ == '__main__':
    name = sys.argv[1]
    filename = os.getcwd() + '/logs/' + str(name) +'.csv'
    df = pd.read_csv(filename)
    copy = df
    copy = copy[(df.T != 0).any()]
    nh_fail = copy[copy.NH_t == 0]
    fnh_fail = copy[copy.FNH_t == 0]
    print 'failures on both implementations: ' + str(df.shape[0] - copy.shape[0])
    print 'failures on backward implementation: ' + str(nh_fail.shape[0])
    print 'failures on forward implementation: ' + str(fnh_fail.shape[0])
    copy = copy[(copy.FNH_t!=0) & (copy.NH_t!=0)]
    print df[(df.FNH_t==0) & (df.NH_t==0)].index
    print copy.describe()
