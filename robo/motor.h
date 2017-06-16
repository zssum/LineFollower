//HelperFunctions.h
 
#ifndef HEADER_MOTOR
  #define HEADER_MOTOR

class Motor{
    public:
    	Motor(int lm1, int lm2, int lm_pwm, int rm1, int rm2, int rm_pwm);
    	void motorFwd(int motorspeed);
  		void motorBack(int motorspeed);
  		void motorLeft(int motorspeed);
  		void motorRight(int motorspeed);
  		void changeSpeed(int leftMotorSpeed, int rightMotorSpeed);
  		void motorStop();
	private:
  		int m_lm1;
  		int m_lm2;
  		int m_lm_pwm;
  		int m_rm1;
  		int m_rm2;
  		int m_rm_pwm;

  };
   
  //Prototype for helper_function found in HelperFunctions.cpp

   
#endif
