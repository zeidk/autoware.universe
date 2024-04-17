import argparse
import csv
import os

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider


def read_csv(directory, filename):
    filepath = os.path.join(directory, filename)
    with open(filepath, "r") as file:
        reader = csv.reader(file)
        data = list(reader)
    return [[float(x) for x in row if x] for row in data]


parser = argparse.ArgumentParser(description="Plot trajectory data from CSV files.")
parser.add_argument("directory", type=str, help="Directory containing the CSV files")
args = parser.parse_args()

data_directory = args.directory

# original_data = read_csv(data_directory, "original_x.log")
resampled_data = read_csv(data_directory, "resampled_x.log")
predicted_data = read_csv(data_directory, "predicted_x.log")
predicted_frenet_data = read_csv(data_directory, "predicted_frenet_x.log")
time_data = read_csv(data_directory, "time.log")

fig, ax = plt.subplots(figsize=(8, 6))
plt.subplots_adjust(bottom=0.2)

# (original_plot,) = ax.plot([], [], marker="o", label="Original Reference Trajectory")
(resampled_plot,) = ax.plot([], [], marker="s", label="Resampled Reference Trajectory")
(predicted_plot,) = ax.plot([], [], marker="d", label="Predicted Trajectory")
(predicted_frenet_plot,) = ax.plot([], [], marker="^", label="Predicted Frenet Trajectory")

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_title("Trajectory Comparison")
ax.legend()
ax.grid(True)

plt.gca().set_aspect("equal", adjustable="box")

slider_ax = plt.axes([0.2, 0.05, 0.6, 0.03])
time_slider = Slider(slider_ax, "Time", 0, len(time_data) - 1, valinit=0, valstep=1)


def update(time_index):
    # original_x = original_data[time_index]
    # original_y = read_csv(data_directory, "original_y.log")[time_index]
    resampled_x = resampled_data[time_index]
    resampled_y = read_csv(data_directory, "resampled_y.log")[time_index]
    predicted_x = predicted_data[time_index]
    predicted_y = read_csv(data_directory, "predicted_y.log")[time_index]
    predicted_frenet_x = predicted_frenet_data[time_index]
    predicted_frenet_y = read_csv(data_directory, "predicted_frenet_y.log")[time_index]

    # original_plot.set_data(original_x, original_y)
    resampled_plot.set_data(resampled_x, resampled_y)
    predicted_plot.set_data(predicted_x, predicted_y)
    predicted_frenet_plot.set_data(predicted_frenet_x, predicted_frenet_y)

    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw_idle()


time_slider.on_changed(update)

update(0)

plt.show()
