import max30102
import hrcalc
import time
import csv

# Initialize sensor
m = max30102.MAX30102()

# Create CSV file and header
filename = "max30102_data.csv"
csv_file = open(filename, mode="w", newline="")
writer = csv.writer(csv_file)

# Write header row
writer.writerow(["Heart Rate", "SpO2"])

print("Collecting data... Press CTRL + C to stop.")

try:
    while True:
        red, ir = m.read_sequential()
        hr, hr_valid, spo2, spo2_valid = hrcalc.calc_hr_and_spo2(ir, red)

        # Only write valid readings
        if hr_valid and spo2_valid:
            print("Heart Rate:", hr, " SpO2:", spo2)
            writer.writerow([hr, spo2])

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped")
    csv_file.close()
    print("CSV file saved as:", filename)
