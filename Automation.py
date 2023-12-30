import pandas as pd
import matplotlib.pyplot as plt

# Constants
SECONDS_PER_HOUR = 3600  # Number of seconds in an hour

# Process times for each station in seconds
process_times = {
    'Loading': 31.91,
    'Processing': 1030.00,
    'Sorting': 3788.1,
    'Unloading': 27.34
}

# Calculate the maximum production rate for each station
max_production_rates = {operation: SECONDS_PER_HOUR / time for operation, time in process_times.items()}

# Identify the bottleneck station (the one with the maximum process time)
bottleneck_station = max(process_times, key=process_times.get)
bottleneck_time = process_times[bottleneck_station]

# Calculate Workload per second (WL/s I) for each station
wl_per_second = {operation: time / SECONDS_PER_HOUR for operation, time in process_times.items()}

# Calculate Utilization (U i) for each station
utilization = {operation: time / bottleneck_time for operation, time in process_times.items()}

# Bottleneck Station Index (BS i) is 1 for the bottleneck station and less than 1 for others
bs_index = {operation: 1 if operation == bottleneck_station else time / bottleneck_time for operation, time in process_times.items()}

# Prepare the data for the DataFrame
data = {
    'Station': list(process_times.keys()),
    'Process Time (s)': list(process_times.values()),
    'WL/s I': list(wl_per_second.values()),
    'U i': list(utilization.values()),
    'BS i': list(bs_index.values()),
    'Max Production Rate (units/h)': list(max_production_rates.values()),
}

# Create a DataFrame
df = pd.DataFrame(data)

# Plotting the data
fig, ax1 = plt.subplots(figsize=(10, 6))

# Bar plot for maximum production rate
ax1.bar(df['Station'], df['Max Production Rate (units/h)'], color='blue', alpha=0.6, label='Max Production Rate (units/h)')
ax1.set_xlabel('Station',fontsize=14)
ax1.set_ylabel('Max Production Rate (units/h)', color='blue',fontsize=14)
ax1.tick_params(axis='y', labelcolor='blue', labelsize=12)

# Instantiate a second axes that shares the same x-axis
ax2 = ax1.twinx()

# Line plot for Utilization and Bottleneck Index
ax2.plot(df['Station'], df['U i'], color='red', marker='o', label='U i (Utilization)', linewidth=2, linestyle='--', alpha=0.6)
ax2.plot(df['Station'], df['BS i'], color='green', marker='x', label='BS i (Bottleneck Station Index)', linewidth=2)
ax2.set_ylabel('Utilization / Bottleneck Index', color='red',fontsize=14)
ax2.tick_params(axis='y', labelcolor='red', labelsize=12)

# Adding legends
ax1.legend(loc='upper left')
ax2.legend(loc='upper right')

# Title of the plot
plt.title('Station Metrics Comparison')

# Show plot
plt.show()