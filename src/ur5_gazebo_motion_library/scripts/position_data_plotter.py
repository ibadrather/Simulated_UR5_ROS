import matplotlib.pyplot as plt
import pandas as pd


def plot_ee_positions(csv_file, start_time=None, end_time=None):
    """
    Plots end-effector positions (x, y, z) from a CSV file with timestamps.

    Args:
        csv_file (str): Path to the CSV file containing the data.
        start_time (float): The start time for the plot (in seconds).
        end_time (float): The end time for the plot (in seconds).
    """
    # Load the CSV data into a pandas DataFrame
    data = pd.read_csv(csv_file)

    # Ensure columns are numeric
    data["Timestamp"] = pd.to_numeric(data["Timestamp"], errors="coerce")
    data["ee_x"] = pd.to_numeric(data["ee_x"], errors="coerce")
    data["ee_y"] = pd.to_numeric(data["ee_y"], errors="coerce")
    data["ee_z"] = pd.to_numeric(data["ee_z"], errors="coerce")

    # Filter data based on the start and end time
    if start_time is not None:
        data = data[data["Timestamp"] >= start_time]
    if end_time is not None:
        data = data[data["Timestamp"] <= end_time]

    if data.empty:
        print("No data available in the specified time range.")
        return

    # Round all numeric values to 2 decimal places
    data = data.round(2)

    # Extract columns and convert to numpy arrays
    timestamps = data["Timestamp"].to_numpy()
    ee_x = data["ee_x"].to_numpy()
    ee_y = data["ee_y"].to_numpy()
    ee_z = data["ee_z"].to_numpy()

    # Create a figure and subplots
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # Plot each position component
    axs[0].plot(timestamps, ee_x, label="ee_x", color="blue")
    axs[0].set_ylabel("Position (m)")
    axs[0].set_title("End-Effector X Position")

    axs[1].plot(timestamps, ee_y, label="ee_y", color="green")
    axs[1].set_ylabel("Position (m)")
    axs[1].set_title("End-Effector Y Position")

    axs[2].plot(timestamps, ee_z, label="ee_z", color="red")
    axs[2].set_ylabel("Position (m)")
    axs[2].set_title("End-Effector Z Position")
    axs[2].set_xlabel("Time (s)")

    # Adjust layout and display the plot
    plt.tight_layout()
    plt.savefig("cropped_position_Data.png", dpi=300)
    plt.show()


# Example usage:
# Replace 'ee_positions.csv' with the path to your CSV file
plot_ee_positions("output_data/ee_positions_24-12-13 Fri 15:33:19.csv", start_time=5.0, end_time=18.0)
