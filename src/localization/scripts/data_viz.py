#!/usr/bin/env python

import os
import glob
import sys
import re
import pandas as pd
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

        # Over-write provided options with defaults
        # (see: https://www.stackabuse.com/how-to-merge-two-dictionaries-in-python/)
        self.options = {**{
            "names": ["ground_truth", "ekf_global", "ekf_local"],
            "save_path": str(os.path.join(os.environ["AUTOMATED_HOME"], "data"))
        }, **options}

        # Plot data
        fig, axes = plt.subplots()
        self.gps_gt.plot(x="x", y="y", ax=axes, xlabel="x", ylabel="y", label=self.options["names"][0])
        self.gps_global.plot(x="x", y="y", ax=axes, xlabel="x", ylabel="y", label=self.options["names"][1])
        self.gps_local.plot(x="x", y="y", ax=axes, xlabel="x", ylabel="y", label=self.options["names"][2])

        # Save figure
        fig_path = self.generate_file_path(os.path.join(self.options["save_path"], "fig.png"))
        plt.savefig(fig_path)

        print("Saved figure to: ", fig_path)

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


if __name__ == "__main__":
    print(sys.argv)
    if len(sys.argv) == 3:
        DataVisualizer(sys.argv[1], sys.argv[2])
    elif len(sys.argv) == 2:
        DataVisualizer(sys.argv[1])
    else:
        print("You suck")
