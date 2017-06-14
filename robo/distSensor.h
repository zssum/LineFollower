//HelperFunctions.h
 
#ifndef HEADER_DISTSENSOR
  #define HEADER_DISTSENSOR

class DistSensor{
    public:
    	DistSensor(int trigPin, int echoPin, int sonicVcc, int sonicGnd);
    	bool rangeIsClear(int distanceInCentimetres);
      long cm;
		
	private:
		long duration;
		int _trigPin, _echoPin, _sonicVcc, _sonicGnd;
  };
   
  //Prototype for helper_function found in HelperFunctions.cpp

   
#endif
