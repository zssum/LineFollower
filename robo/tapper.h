//HelperFunctions.h
 
#ifndef HEADER_TAPPER
  #define HEADER_TAPPER

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

class Tapper{
  public:
    	void initialise(int channelNumber);
      void tap();
      void liftUp();
		
	private:
		Adafruit_PWMServoDriver pwm;
    int servonum;
    
  };
   
  //Prototype for helper_function found in HelperFunctions.cpp

   
#endif
