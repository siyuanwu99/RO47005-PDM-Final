# Copyright (c) Siyuan Wu  All Rights Reserved.
# File Name: %
# Author: Siyuan Wu
# mail: siyuanwu99@gmail.com
# github:https://github.com/edmundwsy
# Created Time: 2021-01-15


import argparse
import os
import re
import numpy
import matplotlib.pyplot as plt

"""
USEAGE
=================================================
```
roslaunch detector debug_djiros_vicon.launch | tee ***.log
python3 running_time_statistician.py -f ***.log
```

"""

parser = argparse.ArgumentParser(description='log reader and time counter')
parser.add_argument('-f', '--filename', default="./logs/log_msgs/0000.log", type=str, metavar="filename", help='filename')
parser.add_argument('-k', '--key', default="elapsed time", type=str, metavar="key", help="key", required=False)
parser.set_defaults(key="elapsed time")
#   parser.add_argument('-h', )



def logreader(filename, key):
    time_buffer = []

    assert os.path.exists(filename), "File does not exist"

    with open(filename) as file:
        for line in file.readlines():
            if key not in line:
                continue
            if 'ms' not in line:
                continue
            l = re.findall('time : ([\d.]+)ms', line)[0]
            time_buffer.append(float(l))

    print("loaded ", filename)

    return time_buffer

def cost_logreader(filename, key):
    cost_buffer = []

    assert os.path.exists(filename), "File does not exist"

    with open(filename) as file:
        for line in file.readlines():
            if 'cost' not in line:
                continue
            l = re.findall('cost : ([\d.]+)', line)[0]
            cost_buffer.append(float(l))

    print("loaded ", filename)

    return cost_buffer

def data_statistics(buffer):
    sum = 0.0
    avg = 0.0
    max = 0.0
    min = 10000
    number = len(buffer)

    if number == 0:
        result = {"mean": avg,
                "min": min,
                "max": max,
                "number": number}
        return result

    for element in buffer:
        sum += element

        if element > max:
            max = element
        
        if element < min:
            min = element
        
    avg = sum/number

    result = {"mean": avg,
            "min": min,
            "max": max,
            "number": number}

    return result
    

def plot_hist(buffer, info):
    
    if info['number'] == 0:
        pass

    assert info['number'] != 0, "no data in this buffer"

    _ = plt.hist(buffer, bins='auto')
    plt.text(0.8 * info['number'], 0.8 * info['max'],
     " Mean %d \n Max %d " % (info['mean'], info['max']))
    plt.show()
    



if __name__ == "__main__":
    args = parser.parse_args()
    time_buf = logreader(args.filename, args.key)
    cost_buf = cost_logreader(args.filename, args.key)
    # print(time_buffer)
    time_stats = data_statistics(time_buf)
    cost_stats = data_statistics(cost_buf)
    print("Time:")
    print(time_stats)
    print("Cost:")
    print(cost_stats)

    # plot_hist(buf, time_stats)