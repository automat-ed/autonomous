#!/usr/bin/env python

import os
import pandas as pd
import matplotlib.pyplot as plt


class DataVisualizer():

    def __init__(self, file_paths, options=None):
        # Save path of all data files
        self.paths = file_paths

        # Over-write provided options with defaults
        # (see: https://www.stackabuse.com/how-to-merge-two-dictionaries-in-python/)
        self.options = {**{
            "names": ["ground_truth", "ekf_global", "ekf_local"],
            "save_path": str(os.path.join(os.environ["AUTOMATED_HOME"], "data"))
        }, **options}

        # To store a list of DataFrames
        self.data_frames = []

        # Read data files
        for file_path in self.paths:
            df = pd.read_csv(file_path)
            self.data_frames.append(df)

        # Plot data
        fig, axes = plt.subplots()
        for i in range(len(self.data_frames)):
            frame = self.data_frames[i]
            frame.plot(x="x", y="y", ax=axes, xlabel="x", ylabel="y", label=options["names"][i])

        # Save figure
        fig_path = os.path.join(self.options["save_path"], "fig.png")
        plt.savefig(fig_path)

        print("Saved figure to: ", fig_path)
