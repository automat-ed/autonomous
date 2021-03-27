#!/usr/bin/env python

import os
import glob
import sys
import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


class DataVisualizer():

    def __init__(self, directory_path, options={}):
        # Define format of files we are looking for
        gps_gt_file_expression = os.path.join(os.path.realpath(directory_path), "gps_gt*.csv")
        gps_global_file_expression = os.path.join(os.path.realpath(directory_path), "gps_global*.csv")
        gps_local_file_expression = os.path.join(os.path.realpath(directory_path), "gps_local*.csv")

        # Get list of files we are working with
        self.gps_gt_files = glob.glob(gps_gt_file_expression)
        self.gps_global_files = glob.glob(gps_global_file_expression)
        self.gps_local_files = glob.glob(gps_local_file_expression)

        # Take average of the data frames
        self.gps_gt = self.calculate_average_frame(self.gps_gt_files)
        self.gps_global = self.calculate_average_frame(self.gps_global_files)
        self.gps_local = self.calculate_average_frame(self.gps_local_files)

        # Store individual errors to plot error over time
        self.errors_local  = []
        self.errors_global = []

        # Calculate average error between gt and ekf_local / ekf_global
        self.error_gt_local, self.error_gt_global = self.calculate_average_error()

        # Over-write provided options with defaults
        # (see: https://www.stackabuse.com/how-to-merge-two-dictionaries-in-python/)
        self.options = {**{
            "names": ["ground truth", "ekf global", "ekf local"],
            "save_path": str(os.path.join(os.environ["AUTOMATED_HOME"], "data"))
        }, **options}

        # Plot data
        fig, axes = plt.subplots()

        # Adjust plot size
        fig.subplots_adjust(bottom=0.27)
        
        self.gps_gt.plot(x="x", y="y", ax=axes, label=self.options["names"][0], color='green', linewidth=2)
        self.gps_local.plot(x="x", y="y", ax=axes, label=self.options["names"][2], linewidth=2)
        self.gps_global.plot(x="x", y="y", ax=axes, label=self.options["names"][1], linewidth=2)

        # Info
        plt.title("Robot Position Estimation", fontweight="bold")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.legend(bbox_to_anchor=(1, 1), loc='upper left')

        # Overlay image of map
        # Square
        # img = plt.imread("./../src/navigation/assets/square.png")
        # axes.imshow(img, origin="lower", extent=[-49, 2, -25, 24], alpha=0.8)

        # Curved Path
        img = plt.imread("./../src/navigation/assets/curved_path.png")
        axes.imshow(img, origin="lower", extent=[-49, 2, -25, 24], alpha=0.8)

        # Display Average Error
        plt.gcf().text(0.44, 0.14, "Average Error", fontweight="bold", fontsize=9)
        plt.gcf().text(0.335, 0.08, 
                      "ekf local: {} m    ekf global: {} m".format(self.error_gt_local, self.error_gt_global),
                       fontsize=8, 
                       bbox=dict(facecolor='none', edgecolor='black', pad=10.0))

        # Save figure
        fig_path = self.generate_file_path(os.path.join(self.options["save_path"], "fig.png"))
        plt.savefig(fig_path)

        print("Saved figure to: ", fig_path)

        # Error over Time Plot
        self.plot_error_over_time()

    def calculate_average_frame(self, file_paths):
        # Get cumulative x and y values
        average_frame = None
        for file_path in file_paths:
            frame = pd.read_csv(file_path)

            if average_frame is None:
                average_frame = frame
            else:
                # Ensure both frames have the same number of rows, trim otherwise
                max_index = min(average_frame.shape[0], frame.shape[0]) - 1
                average_frame = average_frame.truncate(before=0, after=max_index)
                frame = frame.truncate(before=0, after=max_index)

                # Cumulative
                average_frame["x"] += frame["x"]
                average_frame["y"] += frame["y"]

        # Take average
        average_frame["x"] /= len(file_paths)
        average_frame["y"] /= len(file_paths)

        return average_frame

    def generate_file_path(self, path):
        file_path = Path(os.path.expanduser(path))
        while file_path.is_file():
            # Find number
            p = re.compile(r'(?<=\()\d+(?=\).)')
            reg_search = re.search(p, file_path.name)

            if reg_search is None:
                new_name = file_path.stem + "-(1)" + file_path.suffix
            else:
                new_name = re.sub(p, str(int(reg_search.group()) + 1), file_path.name)

            file_path = Path(file_path.parent, new_name)

        return str(file_path)

    def calculate_average_error(self, ):
        ''' Compute error as Euclidian distance between pairs of (x, y)'''
        # Error Ground Truth & EKF Local
        error_gt_local  = 0
        # Error Ground Truth & EKF Global
        error_gt_global = 0
        # Dataframes have same number of observations, pick any one to iterate with index
        for index in range(self.gps_gt.shape[0]):
            point_gt = self.gps_gt.iloc[index, 1:3]

            point_local = self.gps_local.iloc[index, 1:3]
            error_local  = np.linalg.norm(point_local-point_gt)
            error_gt_local += error_local
            # Store individual error for plot of error over time
            self.errors_local.append(error_local)
            
            point_global = self.gps_global.iloc[index, 1:3]
            error_global  = np.linalg.norm(point_global-point_gt)
            error_gt_global += error_global
            # Store individual error for plot of error over time
            self.errors_global.append(error_global)


        error_gt_local /= self.gps_gt.shape[0]
        error_gt_global /= self.gps_gt.shape[0]

        return round(error_gt_local, 3), round(error_gt_global, 3)

    def plot_error_over_time(self, ):
        # Subtract value of first timestep to begin at 0 
        t0 = self.gps_gt["time"][0]
        time = self.gps_gt["time"]
        time -= t0
        # EKF Local error
        fig, ax = plt.subplots()
        ax.plot(time, self.errors_local, label="ekf local error")

        # EKF Global error
        ax.plot(time, self.errors_global, label="ekf global error")

        # Info
        plt.title("Localization error over time", fontweight="bold")
        plt.xlabel("Time (s)")
        plt.ylabel("Error (m)")
        plt.legend()

        # Save plot
        plt.savefig("Error_time")

if __name__ == "__main__":
    if len(sys.argv) == 3:
        DataVisualizer(sys.argv[1], sys.argv[2])
    elif len(sys.argv) == 2:
        DataVisualizer(sys.argv[1])
    else:
        print("You suck")
