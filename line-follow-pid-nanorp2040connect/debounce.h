#pragma once

class Debounce
{
  private:
    int _highThreshold;
    int _lowThreshold;
    bool _triggerOnHigh;
    bool _isHigh;
    uint32_t _debounceStartTime;
    uint32_t _debounceTimeMicros;
    
  public:
    Debounce(int lowThreshold, int highThreshold, bool triggerOnHigh = true, uint32_t debounceTimeMicros = 20000):
      _highThreshold(highThreshold),
      _lowThreshold(lowThreshold),
      _triggerOnHigh(triggerOnHigh),
      _isHigh(false),
      _debounceStartTime(0),
      _debounceTimeMicros(debounceTimeMicros)
    {}

    // Check if the event has just been triggered
    bool isTriggered(int newValue)
    {
      if(_debounceStartTime)
      {
        // Wait for debounce time to pass
        if(micros() - _debounceStartTime >= _debounceTimeMicros)
        {
          // Done debouncing
          _debounceStartTime = 0;
 //         Serial.println("debounce done");
        }
//        else
//          Serial.println("ignored");
      }
      
      if(!_debounceStartTime)
      {
        if(_isHigh)
        {
          if(newValue <= _lowThreshold)
          {
            _isHigh = false;
            _debounceStartTime = micros();
            // Newly low
//            Serial.println("gone low");
            return !_triggerOnHigh;
          }
        }
        else
        {
          if(newValue >= _highThreshold)
          {
            _isHigh = true;
            _debounceStartTime = micros();
            // Newly high
//            Serial.println("gone high");
            return _triggerOnHigh;
          }
        }
      }
      // No change
      return false;
    } 
};
