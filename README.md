# thruster-test-stand
Repository for the OSU UWRT Thruster Test Stand code.

#### Files:
###### ThrusterTestStand.py
 - Run this to log data over serial: `python ThrusterTestStand.py`
   - Make sure the `COM_PORT` variable is set correctly to match the port the Arduino is connected to.
   - Also, be sure to update the `FILE_NAME` variable to avoid overwriting previous logs.

###### ThrusterTestStand.ino
- This goes on the Arduino. Runs the test and logs the data.
- *Note: As of v2.0 this will attempt to log data to the SD card, if no SD card is present it will blink the built-in LED indefinitely.*
  - Test Parameters:
   - `NUM_TEST_POINTS`
     - Defines the number of PWM steps that should be visited in each mode of operation (forward and reverse).
   - `STEP_DURATION`
     - The duration (in milliseconds) each PWM step should last.
   - `SAMPLE_FREQUENCY`
     - The number of samples to take at each PWM step.
   - The rest are self-explanatory, and likely won't often change.
