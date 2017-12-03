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
    COM_PORT = '/dev/ttyACM0'
    print("Let's test some thrusters, shall we? Tell me, which thruster are you testing?")
    FNAME = str(raw_input())
    print("Fantastic choice! I just love " + FNAME + "! Well, let's see if we can't find the Test Stand...\n")
    FNAME = FNAME + ".csv"

    testEnabled = True
    testRunning = False
    dataList = []
    ser = serial.Serial(COM_PORT, baudrate=9600, timeout=None)
    if ser is not None:
        print("Aha! I found it. Now, what are you waiting for? Start the test!\n")
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
                    save(FNAME, dataList)
                    print("Test data saved to " + FNAME + "! It's been a pleasure working with you. Cheers!")
                    sys.exit()
                else:
                    print(data)
                    dataList.append(data)

if __name__ == "__main__":
    main()
