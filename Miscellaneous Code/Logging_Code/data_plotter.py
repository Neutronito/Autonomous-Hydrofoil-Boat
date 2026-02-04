import pandas as pd
import matplotlib.pyplot as plt
import math
import sys
import os
import csv

from matplotlib.ticker import MaxNLocator

def main(filepath):
    print(filepath)
    filename = filepath.split("\\")[-1].split(".")[0]
    print(filename)
    with open(filepath, 'r') as file:
        
        # Setup the iterator which is how we will read lines
        reader = csv.reader(file)

        # Setup the variables which hold our director we read from
        # Get the directory of the opened file
        save_dir = os.path.join(os.path.dirname(filepath), "graphs")
        os.makedirs(save_dir, exist_ok=True)
        
        # Create full path to save the figure in the same folder
        save_path = os.path.join(save_dir, filename + " info.txt")

        # Read the first line (which contains the information)
        current_line = next(reader)
        
        # Create a file to write to
        with open(save_path, "w") as data_file:
            for i in range(0, len(current_line), 2):
                data_file.write(current_line[i].strip())
                data_file.write(": ")
                data_file.write(current_line[i+1].strip())
                data_file.write("\n")


        # Now loop over the 4 PID loops
        for i in range(0, 4, 1):
            # Extract the loop type
            current_line = []
            first_word = ""
            while first_word != "Currently":
                current_line = next(reader)
                if not current_line:
                    continue
                first_word = current_line[0].split(" ")[0]

            # So we are at the top now
            PID_type = current_line[0].split(" ")[-1]

             # Also get the P, I and D values
            PID_gains = (current_line[1].split(" ")[-1], current_line[2].split(" ")[-1], current_line[3].split(" ")[-1], current_line[4].split(" ")[-1], current_line[5].split(" ")[-1], current_line[6].split(" ")[-1] )

            # Skip the header
            throwout = next(reader)

            # Now get the values
            timestamps = []
            sensor_values = []
            setpoints = []
            PID_outputs = []

            #Loop through lines until we get a blank value which means we are at the end
            current_line = next(reader)
            while current_line:
                timestamps.append(int(current_line[0]))
                sensor_values.append(float(current_line[1]))
                setpoints.append(float(current_line[2]))
                PID_outputs.append(float(current_line[3]))
                current_line = next(reader)

            #We are done with getting data
            #Convert the timestamps so they start from 0
            start_time = timestamps[0]
            timestamps = [timestamp - start_time for timestamp in timestamps]

            # If it is a pitch or roll pid, we need to convert sensor and setpoint to degrees
            if PID_type == "Roll" or PID_type == "Pitch" or PID_type == "Yaw":
                sensor_values = [math.degrees(sensor_value) for sensor_value in sensor_values]
                setpoints = [math.degrees(setpoint) for setpoint in setpoints]

            # If it is altitude PID, the setpoint needs to be converted to degrees
            if PID_type == "Altitude":
                PID_outputs = [math.degrees(output) for output in PID_outputs]

            # Now graph it
            plt.rcParams.update({
                'font.size': 22,        # default text size
                'axes.titlesize': 22,   # title size
                'axes.labelsize': 20,   # x/y label size
                'xtick.labelsize': 22,
                'ytick.labelsize': 20,
                'legend.fontsize': 18,
                'font.family': 'serif',
                'font.serif': ['Times New Roman'],
                'font.weight': 'bold',
                'axes.titleweight': 'bold',           # title weight
                'axes.labelweight': 'bold',           # axis label weight
            })
            # Create two vertically stacked plots
            fig, (ax1, ax2) = plt.subplots(nrows=2, sharex=True, figsize=(18, 6), height_ratios=[2, 1])

            # --- Top plot: Sensor and Setpoint ---
            ax1.plot(timestamps, sensor_values, label='Sensor Value', color='black', linewidth=2)
            ax1.plot(timestamps, setpoints, label='Setpoint', linestyle='--', color='green', linewidth=2)
            ax1.set_ylabel("Sensor / Setpoint")
            ax1.legend(loc='upper right')
            ax1.grid(True)

            # --- Bottom plot: PID Output ---
            ax2.plot(timestamps, PID_outputs, label='PID Output', color='red', linewidth=2)
            ax2.set_xlabel("Time (ms)")
            ax2.set_ylabel("PID Output")
            ax2.legend(loc='upper right')
            ax2.grid(True)

            title = (
                f"{PID_type}PID Response Graph, "
                f"P: {float(PID_gains[0]):.5f}, "
                f"I: {float(PID_gains[1]):.5f}, "
                f"D: {float(PID_gains[2]):.5f}, "
                f"I Clamp: {float(PID_gains[3]):.5f} \n"
                f"Parabola Gain: {float(PID_gains[4]):.5f}, "
                f"Parabola Threshold: {float(PID_gains[5]):.3f}"
            )

            ax1.minorticks_on()
            ax2.minorticks_on()

            ax1.yaxis.set_major_locator(MaxNLocator(nbins=4))

            plt.suptitle(title, fontfamily='Times New Roman', fontweight='bold')  # Overall title
            plt.tight_layout(rect=[0, 0, 1, 0.96])  # Leave space for title
            
            

            # Create full path to save the figure in the same folder
            save_path = os.path.join(save_dir, filename + " " + PID_type + ' plot.jpg')

            # Save the plot
            plt.savefig(save_path)

            # Show the plot
            plt.show()

            

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("ERROR, invalid number of arguments. \nUsage: python plot_pid_logs.py <path_to_csv>")
        sys.exit(1)
    csv_path = sys.argv[1]
    if not os.path.exists(csv_path):
        print(f"Error: File '{csv_path}' does not exist.")
        sys.exit(1)
    main(csv_path)
