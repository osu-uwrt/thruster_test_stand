#include "Arduino.h"
#include "DebouncedInput.h"

DebouncedInput::DebouncedInput(int pin, int debounceTime)
{
  _debounceCount = debounceTime;
  _inputPin = pin;
}

bool DebouncedInput::Read()
{
  int reading = 0;
  if(millis() > _previousSampleTime)
  {
   reading = digitalRead(_inputPin);

   if(reading == _debouncedInput && _counter > 0)
   {
     _counter--;
   }
   else
   {
     _counter++;
   }

   // If the Input has shown the same value for long enough let's switch it
   if(_counter >= _debounceCount)
   {
     _counter = 0;
     _debouncedInput = reading;
   }

   _previousSampleTime = millis();
 }

 return _debouncedInput;
}
