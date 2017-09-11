/*
  DebouncedInput.h - Library for debouncing inputs!
  Created by Benji Justice
*/

#ifndef DEBOUNCED_INPUT
#define DEBOUNCED_INPUT

#include "Arduino.h"

class DebouncedInput {
  private:
    int _inputPin;
    int _counter;
    int _debouncedInput;
    long _previousSampleTime;
    int _debounceCount;
  public:
    DebouncedInput(int pin, int debounceTime);
    bool Read();
};

#endif
