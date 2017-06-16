//HelperFunctions.h
 
#ifndef HEADER_TAPPER
  #define HEADER_TAPEPER

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class Tapper{
  public:
    	Tapper(int channelNumber);
      void tap();
      void liftUp();
		
	private:
		Adafruit_PWMServoDriver pwm;
    int servonum;
    
  };
   
  //Prototype for helper_function found in HelperFunctions.cpp

   
#endif
