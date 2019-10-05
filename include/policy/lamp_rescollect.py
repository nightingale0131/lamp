# calculate averages from .dat file
# Author: Florence Tsang
# Creation date: 2019-10-04

import os
import utility as util

if __name__ == '__main__':
    filepath = raw_input('Which file to analyze? Relative from current directory:\n')

    filename, ext = os.path.splitext(filepath)
    if ext != '.dat':
        print('Invalid file!')
        sys.exit()

    policy_data = []
    openloop_data = []
    naive_data = []

    # fill *_data list
    with open(filepath, 'rb') as datafile:
        print('Analyzing {}...'.format(filepath))

        for line in datafile:
            if line.startswith("Policy"):
                parts = line.split()
                policy_data.append(float(parts[1]))
            if line.startswith("Openloop"):
                parts = line.split()
                openloop_data.append(float(parts[1]))
            if line.startswith("Naive"):
                parts = line.split()
                naive_data.append(float(parts[1]))

    print("{:19}: {:.2f}".format("Policy average", util.average(policy_data)))
    print("{:19}: {:.2f}".format("Openloop average", util.average(openloop_data)))
    print("{:19}: {:.2f}".format("Naive average", util.average(naive_data)))
