# -*- coding: utf-8 -*-
"""
decoded custom sensor data from crazyflie
@author: brunojnguerreiro (based on jsschell work)
"""
import cfusdlog
import matplotlib.pyplot as plt
import argparse
import csv
import numpy as np
import os

parser = argparse.ArgumentParser()
parser.add_argument("inputlog")
args = parser.parse_args()

if os.path.isdir(args.inputlog):
    # find all files in the directory that dont end with ".csv" and maintain full path
    logfiles = [os.path.join(args.inputlog, f) for f in os.listdir(args.inputlog) if os.path.isfile(os.path.join(args.inputlog, f)) and not f.endswith(".csv")]
else:
    logfiles = [args.inputlog]

for (idx,file) in enumerate(logfiles):
    print("[Crazyflie uSD dellogger] Found log file",idx+1,"/",len(logfiles),":",file)
    # decode binary log data
    logData = cfusdlog.decode(file)
    # prepare csv file
    outputcsv = os.path.splitext(file)[0] + '.csv'
    csvfile = open(outputcsv,'w', newline='')
    #only focus on regular logging
    logData = logData['fixedFrequency']
    # get keys in current dataset
    keys = list(logData.keys())
    # print("[Crazyflie uSD dellogger] available keys:",keys)
    cvswriter = csv.DictWriter(csvfile, fieldnames=keys)
    cvswriter.writeheader()
    values = np.array(list(logData.values()))
    table = values.transpose()
    for row in table:
        cvswriter.writerow(dict(zip(keys,row)))

    csvfile.close()    
    print("[Crazyflie uSD dellogger] Writing log data to: ",outputcsv)

print("[Crazyflie uSD dellogger] All files processed.")

    

