#ifndef __PIDCONTROLLER_H__
#define __PIDCONTROLLER_H__

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/**
 * PID controller with additional helper functions
 * Features:
 * - PID control
 * - Obstruction detection
 * - Allowed error margin
 * 
 * 
 * Usage:
 * // Define controller (p,i,d,direction)
 * PIDController pid = PIDController(1, 0.1, 0.02, 0);
 *
 * // Set control variables
 * long* InputAngle;
 * long* Output;
 * long* Setpoint;
 * InputAngle = new long(0.0);
 * Output = new long(0.0);
 * Setpoint = new long(0.0);
 * pid.SetControlVariables(long* InputAngle, long* Output, long* Setpoint)
 *
 * // Update input variables
 * *InputAngle = sensorReading();
 *
 * // Compute PID output
 * pid.Compute();
 * 
 * // Use output
 * setMotorDutyCycle(*Output);
 * </p>
 *
 * @author      Wilbert van de Ridder <l.w.vanderidder @ student.utwente.nl>
 * @version     1.0
 * @since       2014-03-21
 *
 * Forked from: https://github.com/br3ttb/Arduino-PID-Library
 * Original license:
 * ***************************************************************
 * * Arduino PID Library - Version 1.1.0
 * 
 * * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * * This Library is licensed under a GPLv3 License
 * 
 * ***************************************************************
 */
class PIDController {
  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1

  //commonly used functions **************************************************************************
	/**
	 * 
	 * @param long P proportional gain
	 * @param long I proportional gain
	 * @param long D proportional gain
	 * @param int Dir direction
	 */
    PIDController(long, long, long, int);     //   Setpoint.  Initial tuning parameters are also set here
	
	void SetControlVariables(long* InputAngle, long* inputVelocity, long* Output, long* Setpoint);
	void SetControlVariables(long* InputAngle, long* Output, long* Setpoint);
	
    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(long, long); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application
	void SetObstructedOutputLimits(long, long);
	


  //available but not commonly used functions ********************************************************
    void SetTunings(long, long,       // * While most users will set the tunings once in the 
                    long);         	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(long);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
	void SetErrorMargin(long);		  // The margin of error allowed around between the setpoint and input		
	void SetObstructionDetection(bool);		
	void SetObstructionVelocityTreshold(long);
	void SetObstructionAngleTreshold(long);
	void SetObstructionTimeTreshold(long);
	void SetObstructionSetpointSensitivity(long);
	void SetObstructed(bool);
	bool GetObstructed();
	bool GetObstructionDetection();			
	void reset();				  
										  
										  
  //Display functions ****************************************************************
	long GetKp();						  // These functions query the pid for interal values.
	long GetKi();						  //  they were created mainly for the pid front-end,
	long GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //
	


  private:
	void Initialize();
	void CheckObstructed();	
	
	long dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	long dispKi;				//   format for display purposes
	long dispKd;				//
    
	long kp;                  // * (P)roportional Tuning Parameter
    long ki;                  // * (I)ntegral Tuning Parameter
    long kd;                  // * (D)erivative Tuning Parameter
	long em;					// Error margin
	int controllerDirection;

    long *myInputAngle;         // * Pointers to the Input, Output, and Setpoint variables
	long *myInputVelocity;      // * Pointers to the Input, Output, and Setpoint variable
    long *myOutput;             //   This creates a hard link between the variables and the 
    long *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
	long lastSetPoint;			  
	unsigned long timeObstructed;
			  
	unsigned long lastTime;
	long ITerm, lastInput;

	unsigned long SampleTime;
	long outMin, outMax, outMinObstructed, outMaxObstructed;
	long obstructionVelocityTreshold;
	long obstructionAngleTreshold;
	long obstructionTimeTreshold;
	long obstructionSetpointSensitivity;
	bool inAuto;
	bool isObstructed;
	bool enableObstructionDetetection;
};
#endif

