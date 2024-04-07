# -*- coding: utf-8 -*-
"""
decoded custom sensor data from crazyflie
@author: brunojnguerreiro (based on jsschell work)
"""
import cfusdlog
import matplotlib.pyplot as plt
import re
import argparse
import csv
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("inputlog")
parser.add_argument("outputcsv")
args = parser.parse_args()

# decode binary log data
logData = cfusdlog.decode(args.inputlog)

# prepare csv file
csvfile = open(args.outputcsv,'w', newline='')
#csvwriter = csv.writer(csvfile)

#only focus on regular logging
logData = logData['fixedFrequency']
# print(logData)

# let's see which keys exists in current data set
keys = list(logData.keys())
# keys = []
# for k, v in logData.items():
#     keys.append(k)

print("keys:",keys)
cvswriter = csv.DictWriter(csvfile, fieldnames=keys)
cvswriter.writeheader()

values = np.array(list(logData.values()))
table = values.transpose()
# print("table:",table)

for row in table:
    # print("row:",row)
    cvswriter.writerow(dict(zip(keys,row)))

# for k, v in logData.items():
#     csvwriter.writerow(v)

csvfile.close()
