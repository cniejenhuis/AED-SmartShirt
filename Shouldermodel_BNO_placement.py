import serial
import time

# Configure the serial connection
ser = serial.Serial('COM3', 9600)  # Update 'COM3' to your Arduino's serial port
time.sleep(2)  # Wait for the connection to be established

# Define the output file name
output_file = 'shouldermovementmovement_versie2_18_6.sto'

# Open the file for writing
with open(output_file, 'w') as file:
    # Write header
    file.write("DataRate=100.000000\n")
    file.write("DataType=Quaternion\n")
    file.write("version=3\n")
    file.write("OpenSimVersion=4.1\n")
    file.write("endheader\n")
    file.write("time\thumerus_r_imu\ttorso_imu\tradius_r_imu\n")
    
    start_time = time.time()
    while True:
        if ser.in_waiting > 0:
            # Read data from serial
            data = ser.readline().decode('utf-8').strip()
            current_time = time.time() - start_time
            # Split data into quaternion components for each sensor
            parts = data.split("\t")
            if len(parts) == 4:
                time_stamp = parts[0]
                humerus_r_imu = parts[1]
                torso_imu = parts[2]
                radius_r_imu = parts[3]
                print(f"{time_stamp}\t{humerus_r_imu}\t{torso_imu}\t{radius_r_imu}")  # Optional: print data to console
                file.write(f"{time_stamp}\t{humerus_r_imu}\t{torso_imu}\t{radius_r_imu}\n")
                file.flush()  # Ensure data is written to file
