# calculate averages from .dat file
# Author: Florence Tsang
# Creation date: 2019-10-04

import os, sys, getopt
import utility as util

def parse_command_line(argv):
    res_file = "lrpp_results.dat"
    n = None

    try:
        opts, args = getopt.getopt(argv, "hf:n:")
    except getopt.GetoptError:
        print("lamp_rescollect.py [-f <file> -n <tasks to process>]")
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print("lamp_rescollect.py [-f <file> -n <tasks to process>]")
            sys.exit()

        elif opt == "-f": res_file = arg.strip()
        elif opt == "-n": n = int(arg)

    return res_file, n

if __name__ == '__main__':
    # filepath = raw_input('Which file to analyze? Relative from current directory:\n')
    filepath, n = parse_command_line(sys.argv[1:])

    filename, ext = os.path.splitext(filepath)
    if ext != '.dat':
        print('Invalid file!')
        sys.exit()

    # policy_data = []
    policy1_data = []
    policy2_data = []
    policy3_data = []
    openloop_data = []
    naive_data = []
    nol = [0,0,0]
    costfn = 0

    # fill *_data list
    with open(filepath, 'rb') as datafile:
        print('Analyzing {}...'.format(filepath))

        for line in datafile:
            '''
            if line.startswith("Policy"):
                parts = line.split()
                policy_data.append(float(parts[1]))
            '''
            if line.startswith("Entered openloop:"):
                # times entered openloop
                if "True" in line:
                    nol[costfn] += 1
                costfn += 1
                if costfn >= 3: costfn = 0
            if line.startswith("Policy 1"):
                parts = line.split()
                policy1_data.append(float(parts[2]))
            if line.startswith("Policy 2"):
                parts = line.split()
                policy2_data.append(float(parts[2]))
            if line.startswith("Policy 3"):
                parts = line.split()
                policy3_data.append(float(parts[2]))
            if line.startswith("Openloop"):
                parts = line.split()
                openloop_data.append(float(parts[1]))
            if line.startswith("Naive"):
                parts = line.split()
                naive_data.append(float(parts[1]))

            if len(naive_data) == n: break

    print("Average over {} tasks:".format(len(naive_data)))
    # print("{:19}: {:.2f}".format("Policy average", util.average(policy_data)))
    print("{:19}: {:.2f}".format("Openloop average", util.average(openloop_data)))
    print("{:19}: {:.2f}".format("Naive average", util.average(naive_data)))
    print("{:19}: {:.2f}".format("Policy 1 average", util.average(policy1_data)))
    print("{:19}: {}".format("  # lambda calls", nol[0]))
    print("{:19}: {:.2f}".format("Policy 2 average", util.average(policy2_data)))
    print("{:19}: {}".format("  # lambda calls", nol[1]))
    print("{:19}: {:.2f}".format("Policy 3 average", util.average(policy3_data)))
    print("{:19}: {}".format("  # lambda calls", nol[2]))
