//HelperFunctions.h
 
#ifndef HEADER_MOTOR
  #define HEADER_MOTOR

class Motor{
    public:
    	Motor(int lm1, int lm2, int lm_pwm, int rm1, int rm2, int rm_pwm, int m_GRD, int m_5V);
    	void motorFwd(int motorspeed);
  		void motorBack(int motorspeed);
  		void motorLeft(int motorspeed);
  		void motorRight(int motorspeed);
  		void changeSpeed(int leftMotorSpeed, int rightMotorSpeed);
      void softAccelerateToSpeed(int leftMotorSpeed, int rightMotorSpeed);
      void softBrakeFromSpeed();
  		void motorStop();
	private:
  		int m_lm1;
  		int m_lm2;
  		int m_lm_pwm;
  		int m_rm1;
  		int m_rm2;
  		int m_rm_pwm;
      int motorCorrection=10;

  };
   
  //Prototype for helper_function found in HelperFunctions.cpp

   
#endif
