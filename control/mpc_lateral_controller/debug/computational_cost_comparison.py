import argparse
import os
import re

import matplotlib.pyplot as plt


def find_latest_log_file(directory):
    log_files = [
        f
        for f in os.listdir(directory)
        if f.startswith("test_trajectory_follower_node_") and f.endswith(".log")
    ]
    if not log_files:
        raise FileNotFoundError("No log files found in the specified directory.")
    return max(log_files, key=lambda f: int(f.split("_")[-1].split(".")[0]))


def extract_calculation_times(log_file_path):
    with open(log_file_path, "r") as file:
        log_data = file.read()

    osqp_pattern = r"\[controller\.lateral_controller\]: executeOptimization time = (\d+) \[ns\]"
    cgmres_pattern = (
        r"\[controller\.lateral_controller\]: executeOptimization \(cgmres\) time = (\d+) \[ns\]"
    )

    osqp_times = re.findall(osqp_pattern, log_data)
    cgmres_times = re.findall(cgmres_pattern, log_data)

    osqp_times = [int(time) / 1000000 for time in osqp_times]  # Convert nsec to msec
    cgmres_times = [int(time) / 1000000 for time in cgmres_times]  # Convert nsec to msec

    return osqp_times, cgmres_times


def plot_calculation_times(osqp_times, cgmres_times, log_file_name):
    plt.figure(figsize=(10, 6))
    plt.plot(range(len(osqp_times)), osqp_times, label="OSQP")
    plt.plot(range(len(cgmres_times)), cgmres_times, label="CGMRES")
    plt.xlabel("Iteration")
    plt.ylabel("Calculation Time [ms]")  # Change the unit to msec
    plt.title(f"OSQP vs CGMRES Calculation Time\nLog File: {log_file_name}")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot OSQP and CGMRES calculation times from a log file."
    )
    parser.add_argument("log_directory", help="Directory containing the log files.")
    parser.add_argument(
        "-f",
        "--file",
        help="Specify the log file name. If not provided, the latest log file will be used.",
    )
    args = parser.parse_args()

    if args.file:
        log_file_name = args.file
        log_file_path = os.path.join(args.log_directory, log_file_name)
    else:
        log_file_name = find_latest_log_file(args.log_directory)
        log_file_path = os.path.join(args.log_directory, log_file_name)

    osqp_times, cgmres_times = extract_calculation_times(log_file_path)
    plot_calculation_times(osqp_times, cgmres_times, log_file_name)
