import csv
import re  # Import the regular expression library
import time

import serial

# Regex to find "RED: <number>" and "IR: <number>"
# \d+ means "one or more digits"
# \s* means "zero or more whitespace characters"
line_regex = re.compile(r"RED:\s*(\d+)\s+IR:\s*(\d+)")

ser = serial.Serial('COM10', 115200, timeout=1) 

with open('ppg_sensor_data.csv', mode='a', newline='') as file:
    writer = csv.writer(file)

    if file.tell() == 0:
        writer.writerow(['RED', 'IR'])

    print("Connected to ESP32. Saving data...")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            line = ser.readline().decode('utf-8').strip()

            if line:
                # Use regex to find a match
                match = line_regex.search(line)
                
                if match:
                    # group(1) is the first number (RED)
                    # group(2) is the second number (IR)
                    red = match.group(1)
                    ir = match.group(2)
                    
                    writer.writerow([red, ir])
                    print(f"RED: {red}, IR: {ir}")
                else:
                    print(f"Skipping line (no match): {line}")

            # NO time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping data saving...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ser.close()
        print("Serial port closed. File saved.")