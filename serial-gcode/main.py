import argparse
import serial
import time


def send_gcode(file_path, com_port):
    # Open the serial connection
    ser = serial.Serial(com_port, 115200, timeout=1)

    # Open the Gcode file
    with open(file_path, 'r') as f:
        # Read the lines of the file
        lines = f.readlines()

        counter = 0
        # Send each line over the serial connection
        for line in lines:
            # Strip any newline characters from the line
            line = line.strip()

            # Send the line over the serial connection
            ser.write((line + '\n').encode())

            counter += 1
            if counter == 190:
                counter = 0
                time.sleep(2)

    # Close the serial connection
    ser.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Send Gcode file over a COM port.')
    parser.add_argument('file_path', help='The path to the Gcode file.')
    parser.add_argument('com_port', help='The COM port to use (e.g. COM3 or /dev/ttyUSB0).')
    args = parser.parse_args()

    send_gcode(args.file_path, args.com_port)
