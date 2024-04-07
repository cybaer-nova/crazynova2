# -*- coding: utf-8 -*-
"""
decoded custom ros2bag files from crazyflie experiments
only works on linux due to dependences on RCLPY and others
@author: brunojnguerreiro (based on https://github.com/fishros/ros2bag_convert/)
"""

import argparse
import sqlite3
import numpy as np
import read_bag
import save_csv_file
import os

# parse arguments
parser = argparse.ArgumentParser(description='Convert crazyflie rosbags to csv file.')
parser.add_argument("inputbag",help="input rosbag file")
args = parser.parse_args()

# Connect to the ros2bag database
bag_file = args.inputbag
# print("bag file:",bag_file)
connection = sqlite3.connect(bag_file)
cursor = connection.cursor()

# Get all topics names and types
print_out=False
topic_names = read_bag.get_all_topics_names(cursor, print_out)
print("Available topic names in bag file:")
print(topic_names)
topic_types = read_bag.get_all_msgs_types(cursor, print_out)

# Get all timestamps and messages for each topic
for i in range(len(topic_names)):
    timestamps, messages = read_bag.read_from_topic(bag_file, topic_names[i], print_out)
    topic_name,topic_type = topic_names[i],topic_types[i]
    tmp = topic_name.split("/")
    path, topic_name = "/".join(tmp[:-1]),tmp[-1]
    path = bag_file[:bag_file.rfind("/")]+""+path
    os.makedirs(path, exist_ok=True)
    save_csv_file.save_csv_file([timestamps, messages],path + "/" + topic_name+".csv")

# Close connection to the database
connection.close()


