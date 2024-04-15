import argparse
import csv
import os

import matplotlib.pyplot as plt


def read_csv(directory, filename):
    filepath = os.path.join(directory, filename)
    with open(filepath, "r") as file:
        reader = csv.reader(file)
        data = list(reader)
    return [float(x) for x in data[0] if x]


parser = argparse.ArgumentParser(description="Plot trajectory data from CSV files.")
parser.add_argument("directory", type=str, help="Directory containing the CSV files")
args = parser.parse_args()

data_directory = args.directory

original_x = read_csv(data_directory, "original_x.log")
original_y = read_csv(data_directory, "original_y.log")
resampled_x = read_csv(data_directory, "resampled_x.log")
resampled_y = read_csv(data_directory, "resampled_y.log")
predicted_x = read_csv(data_directory, "predicted_x.log")
predicted_y = read_csv(data_directory, "predicted_y.log")

# グラフの描画
plt.figure(figsize=(8, 6))
plt.plot(original_x, original_y, marker="o", label="Original Reference Trajectory")
plt.plot(resampled_x, resampled_y, marker="s", label="Resampled Reference Trajectory")
plt.plot(predicted_x, predicted_y, marker="d", label="Predicted Trajectory")
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Trajectory Comparison")
plt.legend()
plt.grid(True)
plt.show()
