import serial
import time
import csv
import sys

def save(filename, data):
    with open(filename, 'wb') as file:
        file.write('Time(ms),PWM(um),Measured Force(lbs)\n')
        for row in data:
            file.write(row)

def main():
    COM_PORT = '/dev/ttyACM1'
    FILE_NAME = 'thrust.csv'

    testEnabled = True
    testRunning = False
    dataList = []
    ser = serial.Serial(COM_PORT, baudrate=9600, timeout=None)
    if ser is not None:
        print("Found device. Awaiting test start...\n")
        while testEnabled:
            data = ser.readline()
            data = data[:-1]

            if len(data) == 2:
                data = data[0]

            if data is not "":
                if (data == "$" and not testRunning):
                    print('Test started!')
                    testRunning = True
                elif (data == '!' and testRunning):
                    testRunning = False
                    testEnabled = False
                    print('Test finished. Saving...')
                    save(FILE_NAME, dataList)
                    sys.exit()
                else:
                    print(data)
                    dataList.append(data)

if __name__ == "__main__":
    main()
