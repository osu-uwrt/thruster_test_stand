// Thruster Test Stand v2.0
// Author: Benji Justice (justice.251@osu.edu)
//
// The Ohio State University
// Underwater Robotics Team

// Uncomment the following line to enable serial output
//#define SERIAL_DEBUG

#include <Servo.h>              // Arduino servo library for sending PWM
#include <DebouncedInput.h>     // Custom library for debounced inputs
#include <SD.h>                 // Arduino SD card library for datalogging
#define SERIAL_DEBUG
/****************************
 * TEST & CIRCUIT PARAMETERS
 ****************************/

// Test parameters
int NUM_TEST_POINTS = 20; // Number of PWMs to test in each operation mode (forward/reverse)
long STEP_DURATION = 2000; // Duration of each PWM step (how long we write each PWM), milliseconds
int SAMPLE_FREQUENCY = 3; // Number of samples to take at each PWM step

// Arduino parameters
float ARDUINO_VOLT_PER_BIT = (5.0/1023.0); // Arduino has a 10-byte 5V ADC (V/Bit)

// Pin Definitions
int ARM_SWITCH_PIN = 2;
int START_BUTTON_PIN = 3;
int STOP_BUTTON_PIN = 4;
int LOAD_CELL_ANALOG_PIN = 0;
int SD_SHIELD_PIN = 10;

// Load cell calibration parameters
float LOAD_CELL_LB_PER_VOLT = (50.0 / 4.0); // Pounds per volt
float LOAD_CELL_VOLT_OFFSET = 0.5; // Voltage at 0 force
float LOAD_CELL_FORCE_OFFSET = 26.25; // Initial force on the load cell

// Thruster PWM constants
int ZERO_PWM = 1500;
int MAX_PWM = 1700;
int MIN_PWM = 1300;

/*************
 * VARIABLES
 *************/

// Serial variables
int SERIAL_BAUD_RATE = 9600;
char START_BYTES = '$'; // Bytes signaling the start of the test.
char END_BYTES = '!'; // Bytes signaling the end of the test.

int _currentPWM = ZERO_PWM; // Keeps track of the PWM currently on the ESC
int _deltaPWM = (MAX_PWM - ZERO_PWM) / NUM_TEST_POINTS; // Amount to change PWM between steps
bool _testRunning = false; // Keeps track of whether or not the test is running

// _currentPeriod: Enumeration of the section of the test we're on.
// 0 - increasing PWM until MAX_PWM
// 1 - decreasing PWM until MIN_PWM
// 2 - increasing PWM until ZERO_PWM
// Resets at end of test.
int _currentPeriod = 0;

// File object to write data to
File logfile;

// Timing variables
long _previousStepEndTime; // Time the last PWM step ended
long _previousSampleTime; // Time the last sample was taken
long _samplePeriod = STEP_DURATION / SAMPLE_FREQUENCY; // Time between samples

// Debounced inputs (pin, debouncetime)
DebouncedInput _armSwitch = DebouncedInput(ARM_SWITCH_PIN, 10);
DebouncedInput _startButton = DebouncedInput(START_BUTTON_PIN, 10);
DebouncedInput _stopButton = DebouncedInput(STOP_BUTTON_PIN, 10);

// Thruster variables
Servo _thruster;
int THRUSTER_PIN = 9;

// Function prototypes
void readAndLog();
void testThruster();
void stopTest();
void startTest();
void error(char* msg);

/***************/
/* Setup       */
/***************/
void setup() {
  Serial.begin(SERIAL_BAUD_RATE); // Establish serial connection

  //pinMode(SD_SHIELD_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
/*
  // Setup the SD card
  if (!SD.begin(SD_SHIELD_PIN)) {
    error("Card failed, or not present.");
  }

  // Create new file on SD card
  // There is some logic in here to increment the filename if a previous file exists
  // TODO: These files should be named for each thruster eventually, perhaps
  //       set a DIP switch or something on the box.
  char filename[] = "TESTSTAND00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }

  // Don't continue without that file
  if (!logfile) {
    error("Couldn't create log file.");
  }

  // Print log file header
  logfile.println("time,PWM,force");
*/
  _thruster.attach(THRUSTER_PIN);  // Tell Arduino what pin the thruster is on
  _thruster.writeMicroseconds(ZERO_PWM); // Write 0 pwm to thruster on start up
  delay(1000); // Wait a sec.

  // Setup arm switch and buttons
  pinMode(ARM_SWITCH_PIN, INPUT);
  pinMode(START_BUTTON_PIN, INPUT);
  pinMode(STOP_BUTTON_PIN, INPUT);
}


/***************/
/* Main Loop   */
/***************/
void loop(){

  // Read the arm switch :)
  bool testArmed = _armSwitch.Read();

  // This is dense, but not bad.
  // If the test is armed, we check whether the test needs to be stopped, started, or updated.
  // If the test is not armed, and the test was running, we need to stop the test.
  if (testArmed) {
    if (_testRunning) {
      if (_stopButton.Read()) {
          // Kill because stop button
          stopTest();
      } else {
        // Update the test
        testThruster(millis());
      }
    } else if (!_testRunning && _startButton.Read()) {
      // BEGIN ZE TEST!!!
      startTest();
    }
  } else if (_testRunning && !testArmed) {
    // Kill because disarmed
    stopTest();
  }
}

// Begins the test. Prints the start bytes over serial.
void startTest() {
  _testRunning = true;
  _previousStepEndTime = millis();
#ifdef SERIAL_DEBUG
  Serial.println(START_BYTES);
#endif
}

// Stops the thruster and resets the test variables
void stopTest() {
  // STOP IT!!!
  _thruster.writeMicroseconds(ZERO_PWM);
  delay(1000);

  // Reset ALL the things.

  _currentPeriod = ZERO_PWM;
  _currentPWM = ZERO_PWM;
  _testRunning = false;

#ifdef SERIAL_DEBUG
  Serial.println(END_BYTES);
#endif
}

// Runs the thruster, sets PWMs, and logs the data based on the currentTime.
// The switch statement in here basically defines what the test is.
// Might be nice to have this based on a function or something in future so
// it's less clunky and easier to modify the "shape" of the test.
void testThruster(long currentTime) {
  // Is it time to record the force? If so, record the force.
  if (currentTime - _previousSampleTime >= _samplePeriod) {
    readAndLog(currentTime);
    _previousSampleTime = currentTime;
  }

  // Is it time to change the PWM? If so, change the PWM.
  if (currentTime - _previousStepEndTime >= STEP_DURATION) {
    _previousStepEndTime = currentTime;

    // Move to next PWM or period if need be
    switch(_currentPeriod) {
      case 0:
        if (_currentPWM >= MAX_PWM)
          _currentPeriod++;
        else
          _currentPWM += _deltaPWM;
        break;
      case 1:
        if (_currentPWM <= MIN_PWM)
          _currentPeriod++;
        else
          _currentPWM -= _deltaPWM;
        break;
      case 2:
        if (_currentPWM >= ZERO_PWM)
        {
            stopTest();
        }
        else
          _currentPWM += _deltaPWM;
        break;
    }

    // Write new PWM to ESC
    _thruster.writeMicroseconds(_currentPWM);
  }
}

void error(char* msg) {
  Serial.println(msg);
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

// Prints the force, PWM, and time over serial
// Eventually we'll read current and temperature etc
void readAndLog(long time) {
  // Read force
  int senseValue = analogRead(LOAD_CELL_ANALOG_PIN);

  // Read force
  float loadCellVoltage = senseValue * ARDUINO_VOLT_PER_BIT;
  float loadCellForce = ((loadCellVoltage - LOAD_CELL_VOLT_OFFSET) * LOAD_CELL_LB_PER_VOLT) - LOAD_CELL_FORCE_OFFSET;
/*
  logfile.print(time);
  logfile.print(',');
  logfile.print(_currentPWM);
  logfile.print(',');
  logfile.println(loadCellForce);
*/
#ifdef SERIAL_DEBUG
  // Transmit data over serial
  Serial.print(time);
  Serial.print(',');
  Serial.print(_currentPWM);
  Serial.print(',');
  Serial.println(loadCellForce);
#endif
}
