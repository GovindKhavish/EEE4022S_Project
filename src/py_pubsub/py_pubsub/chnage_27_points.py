import csv
import os

# Input CSV file path and output CSV file path
input_folder = "/home/riser14/Test"
output_folder = "/home/riser14/ros_ws"
input_file = "odom_true_path.csv"
output_file = "reference_path.csv"

# Function to read and write CSV data
def filter_and_write_csv(input_path, output_path, step=105):
    with open(input_path, 'r') as csv_in, open(output_path, 'w', newline='') as csv_out:
        reader = csv.reader(csv_in)
        writer = csv.writer(csv_out)
        
        # Write header
        header = next(reader)
        writer.writerow(header)
        
        # Initialize a counter to keep track of the rows
        count = 0

        for row in reader:
            count += 1
            if count % step == 0:
                writer.writerow(row)

if __name__ == "__main__":
    input_path = os.path.join(input_folder, input_file)
    output_path = os.path.join(output_folder, output_file)

    filter_and_write_csv(input_path, output_path)
    print(f"Every 108th row from {input_path} has been written to {output_path}.")