// Use Arduino servo library to control the ESC speed controller
#include <Servo.h>

/****************************
 * TEST & CIRCUIT PARAMETERS
 ****************************/

// Test parameters
int NUM_TEST_POINTS = 5;
long STEP_DURATION = 2000; // milliseconds
int SAMPLE_FREQUENCY = 3; // samples per step

// Arduino parameters
float ARDUINO_VOLT_PER_DIV = (5.0/1023.0);
int START_SWITCH_PIN = 2;
int LOAD_CELL_ANALOG_PIN = 0;

// Load cell parameters
float LOAD_CELL_LB_PER_VOLT = (50.0 / 4.0);
float LOAD_CELL_VOLT_OFFSET = 0.5;
float LOAD_CELL_FORCE_OFFSET = 21.57; // Initial force on the load cell

// Thruster PWM values
int STOP = 1500;
int MAX_PWM = 1900;
int MIN_PWM = 1100;


/*************
 * VARIABLES
 *************/

// Serial variables
int SERIAL_BAUD_RATE = 9600;
char START_BYTES = '$';
char END_BYTES = '!';

int currentPWM = STOP;
int deltaPWM;
bool forwardThrust = true;
bool testRunning = false;
bool awaitingStart = true;

// Timing variables
long testStartMillis;
long testEndMillis;
long stepStartMillis;
long lastStepEndMillis;
long lastSampleTime;
long sampleTime;

Servo thruster;
int THRUSTER_PIN = 9;

// Switch Debounce Vars
int counter = 0;       // how many times we have seen new value
int reading;           // the current value read from the input pin
int currentStartSwitch = LOW;    // the debounced input value

long debounceTime = 0;         // the last time the output pin was sampled
int debounceCount = 10; // number of millis/samples to consider before declaring a debounced input

// Function prototypes
void readAndLog();
void testThruster();
void resetTest();
bool readStartSwitchDebounced();

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  deltaPWM = (MAX_PWM - STOP) / NUM_TEST_POINTS;
  sampleTime = STEP_DURATION / SAMPLE_FREQUENCY;

  thruster.attach(THRUSTER_PIN);
  thruster.writeMicroseconds(STOP);
  delay(1000);

  // Setup start switch
  pinMode(START_SWITCH_PIN, INPUT);
}

void loop(){
  bool startSwitch = readStartSwitchDebounced();
  if (awaitingStart) {
    if (startSwitch) {
      awaitingStart = false;

      // Send start signal
      Serial.println(START_BYTES);

      testRunning = true;
      testStartMillis = millis();
      lastStepEndMillis = testStartMillis;

    }
  }
  else if (testRunning) {
    if (!startSwitch) {
      // Send STOP to thruster
      thruster.writeMicroseconds(STOP);
      delay(1000);
      // Send end bytes
      Serial.println(END_BYTES);
      testRunning = false;
      resetTest();
    }
  }
  else if (!awaitingStart) {
    if (!startSwitch) {
      awaitingStart = true;
    }
  }

  if (testRunning) {
    testThruster(millis());
  }
}

bool readStartSwitchDebounced() {
  bool val = currentStartSwitch;
  if(millis() != debounceTime)
  {
    reading = digitalRead(START_SWITCH_PIN);

    if(reading == currentStartSwitch && counter > 0)
    {
      counter--;
    }
    if(reading != currentStartSwitch)
    {
      counter++;
    }
    // If the Input has shown the same value for long enough let's switch it
    if(counter >= debounceCount)
    {
      counter = 0;
      currentStartSwitch = reading;
      val = currentStartSwitch;
    }
    debounceTime = millis();
  }
  return val;
}

void resetTest() {
  currentPWM = STOP;
  forwardThrust = true;
}

void testThruster(long currentTime) {
  // Is it time for a sample? If so, take a sample.
  if (currentTime - lastSampleTime >= sampleTime) {
    lastSampleTime = currentTime;
    readAndLog(currentTime);
  }

  // Is it time to change the thrust? If so, change the thrust.
  if (currentTime - lastStepEndMillis >= STEP_DURATION) {
    lastStepEndMillis = currentTime;

    // Move to next PWM
    if (currentPWM >= MAX_PWM && forwardThrust) {
      forwardThrust = false;
    }
    else if (currentPWM <= MIN_PWM) {
      testRunning = false;
      testEndMillis = millis();
      currentPWM = STOP;
      thruster.writeMicroseconds(STOP);
      delay(1000);
      forwardThrust = true;
      Serial.println(END_BYTES);
    }

    if (forwardThrust && testRunning) {
      currentPWM += deltaPWM;
    }
    else {
      currentPWM -= deltaPWM;
    }

    // Write new PWM to ESC
    thruster.writeMicroseconds(currentPWM);
  }
}

// Logs the force and PWM at a given time
void readAndLog(long time) {
  // Read force
  int senseValue = analogRead(LOAD_CELL_ANALOG_PIN);

  // Log PWM and force
  float loadCellVoltage = senseValue * ARDUINO_VOLT_PER_DIV;
  float loadCellForce = ((loadCellVoltage - LOAD_CELL_VOLT_OFFSET) * LOAD_CELL_LB_PER_VOLT) - LOAD_CELL_FORCE_OFFSET;
  // Transmit data over serial
  //char* data;
  //sprintf(data, "%u, %d, %f", time, currentPWM, loadCellForce);
  Serial.print(time);
  Serial.print(',');
  Serial.print(currentPWM);
  Serial.print(',');
  Serial.println(loadCellForce);

}
