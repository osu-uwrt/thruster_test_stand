import serial
import time
import csv
import sys

def save(data):
    with open('heave_stbd_aft.csv', 'wb') as file:
        file.write('Time(ms),PWM(um),Measured Force(lbs)\n')
        for row in data:
            file.write(row)

def main():
    testEnabled = True
    testRunning = False
    dataList = []
    ser = serial.Serial('/dev/ttyACM1', baudrate=9600, timeout=None)
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
                    save(dataList)
                    sys.exit()
                else:
                    print(data)
                    dataList.append(data)

if __name__ == "__main__":
    main()
