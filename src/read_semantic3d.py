#!/usr/bin/env python
#coding:utf-8
import pptk
import pandas as pd
import numpy as np
import argparse
import rospy
import csv


def read_points(f):
    col_names = ['x', 'y', 'z', 'intensity', 'r', 'g', 'b']
    col_dtype = {'x': np.float32, 'y': np.float32, 'z': np.float32, 'intensity': np.int32, 'r': np.uint8, 'g': np.uint8, 'b': np.uint8}
    return pd.read_csv(f, names=col_names, dtype=col_dtype, delim_whitespace=True)


def read_labels(f):
    return pd.read_csv(f, header=None)[0].values


def main(args):
    print(__file__ + " start!")
    print('semantic3d')

    f_pc = open(args.pc_data_path, 'r')
    f_label = open(args.label_data_path, 'r')
    points = read_points(f_pc)
    labels = read_labels(f_label)
    
    f_out = open('segmented_pc.csv','w')
    writer=csv.writer(f_out)
    for i in range(len(labels)):
        print(i)
        writer.writerow([points['x'][i], points['y'][i], points['z'][i], points['intensity'][i], points['r'][i], points['g'][i], points['b'][i], labels[i]])
    
    print(points.shape)
    print(labels.shape)
    print(labels[0])
    print(points['b'][0])

    f_pc.close()
    f_label.close()
    f_out.close()   


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--pc_data_path', type=str, default='/home/amsl/ros_catkin_ws/src/test_repository/src/dataset/semantic3d/bildstein_station1_xyz_intensity_rgb.txt')
    parser.add_argument('--label_data_path', type=str, default='/home/amsl/ros_catkin_ws/src/test_repository/src/dataset/semantic3d/bildstein_station1_xyz_intensity_rgb.labels')
    

    args = parser.parse_args()

    main(args)



