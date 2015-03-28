#include "PID_Extended_Controller.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PIDController::PIDController(long Kp, long Ki, long Kd, int ControllerDirection) {
	inAuto = false;
	
	SetOutputLimits(0, 255);				//default output limit corresponds to 
												//the arduino pwm limits

    SampleTime = 100;							//default Controller Sample Time is 0.1 seconds
	em = 0;										// Default error margin
	
    SetControllerDirection(ControllerDirection);
    SetTunings(Kp, Ki, Kd);

    lastTime = millis()-SampleTime;		
	
	// Obstruction detection defaults
	SetObstructedOutputLimits(-255, 255);
	isObstructed = false;
	enableObstructionDetetection = false;	
	obstructionVelocityTreshold = 0;
	obstructionAngleTreshold = 0;
	obstructionTimeTreshold = 0;		
	obstructionSetpointSensitivity = 0;			
}
 
void PIDController::SetControlVariables(long* InputAngle, long* inputVelocity, long* Output, long* Setpoint) {
    myOutput = Output;
    myInputAngle = InputAngle;
	myInputVelocity = inputVelocity;
    mySetpoint = Setpoint;
}

void PIDController::SetControlVariables(long* InputAngle, long* Output, long* Setpoint) {
	myOutput = Output;
	myInputAngle = InputAngle;
	mySetpoint = Setpoint;
	enableObstructionDetetection = false;
}
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool PIDController::Compute()
{
	if(!inAuto) return false;
	unsigned long now = millis();
	unsigned long timeChange = (now - lastTime);
	
	// Check for obstruction
	if (enableObstructionDetetection) {
		CheckObstructed();
	}
	
	if(timeChange >= SampleTime)
	{
		// Compute all the working error variables
		long input = *myInputAngle;
		long error = *mySetpoint  - input;
	  
		// Check error margin
		if (abs(error) <  em) {
			error = 0;
		}
	  
		// Calculate terms
		ITerm+= (ki * error);
		if(ITerm > outMax) ITerm= outMax;
		else if(ITerm < outMin) ITerm= outMin;
		long dInput = (input - lastInput);
 
		// Compute PID Output
		long output = (kp * error) + ITerm - (kd * dInput);
      
		// Clamp output
		if (isObstructed && enableObstructionDetetection) {
			if(output > outMaxObstructed) output = outMaxObstructed;
			else if(output < outMinObstructed) output = outMinObstructed;
		}
		if(output > outMax) output = outMax;
		else if(output < outMin) output = outMin;
		*myOutput = output;
	  
		// Remember some variables for next time
		lastInput = input;
		lastTime = now;
	  return true;
   }
   else return false;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PIDController::SetTunings(long Kp, long Ki, long Kd)
{
   if (Kp == dispKp && Ki == dispKi && Kd == dispKd) return;
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   dispKp = Kp; dispKi = Ki; dispKd = Kd;
   
   double SampleTimeInSec = ((double)SampleTime)/1000;  
   kp = Kp;
   ki = (Ki * SampleTimeInSec);
   kd = (Kd / SampleTimeInSec) ;
 
  if(controllerDirection == REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PIDController::SetSampleTime(long NewSampleTime)
{
   if (NewSampleTime > 0 && NewSampleTime != SampleTime)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PIDController::SetOutputLimits(long Min, long Max)
{
   if(Min >= Max && (Min != 0 || Max != 0) || (Min == outMin && Max == outMax)) return;
   outMin = Min;
   outMax = Max;
 
   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;
	 
	   if(ITerm > outMax) ITerm= outMax;
	   else if(ITerm < outMin) ITerm= outMin;
   }
}

void PIDController::CheckObstructed() {
   // Calculate time variables
   unsigned long timeChange = millis() - lastTime;
   
   // Check if currently obstructed
   if (abs(*myInputVelocity) <= obstructionVelocityTreshold && abs(*myInputAngle - *mySetpoint) > obstructionAngleTreshold && abs(*myOutput) > 0) {//&& abs(lastSetPoint - *mySetpoint) <= obstructionSetpointSensitivity) {
	   // It may be obstructed, check if time treshold has been reached
	   // Calculate time obstructed
	   timeObstructed += timeChange;
	   if (timeObstructed >= obstructionTimeTreshold) {
		   isObstructed = true;
	   }
	   else {
		   isObstructed = false;
	   }	   
   }
   else {// if (timeObstructed || isObstructed) {
	   // It's not, reset time and set obstruction indication to false.   
	   timeObstructed = 0;
	   isObstructed = false;
   }
   lastSetPoint = *mySetpoint;
}

void PIDController::SetObstructedOutputLimits(long Min, long Max)
{
   if(Min >= Max) return;
   outMinObstructed = Min;
   outMaxObstructed = Max;
 
   if(inAuto && isObstructed)
   {
	   if(*myOutput > outMaxObstructed) *myOutput = outMaxObstructed;
	   else if(*myOutput < outMinObstructed) *myOutput = outMinObstructed;
	 
	   if(ITerm > outMaxObstructed) ITerm= outMaxObstructed;
	   else if(ITerm < outMinObstructed) ITerm= outMinObstructed;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PIDController::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}

/* SetErrorMargin(...)****************************************************************
 * Sets the error margin
 ******************************************************************************/ 
void PIDController::SetErrorMargin(long Em)
{
    em = Em;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PIDController::Initialize()
{
   ITerm = *myOutput;
   lastInput = *myInputAngle;
   if(ITerm > outMax) ITerm = outMax;
   else if(ITerm < outMin) ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PIDController::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   controllerDirection = Direction;
}

void PIDController::SetObstructionDetection(bool detect)
{
   enableObstructionDetetection = detect;
   
   if (!detect) {
	   isObstructed = false;
   }
}

void PIDController::SetObstructionVelocityTreshold(long treshold)
{
   obstructionVelocityTreshold = treshold;
}

void PIDController::SetObstructionAngleTreshold(long treshold)
{
   obstructionAngleTreshold = treshold;
}

void PIDController::SetObstructionTimeTreshold(long treshold)
{
   obstructionTimeTreshold = treshold;
}

void PIDController::SetObstructionSetpointSensitivity(long sensitivity)
{
   obstructionSetpointSensitivity = sensitivity;
}

void PIDController::SetObstructed(bool obstructed)
{
   isObstructed = obstructed;
   
   if (!obstructed) {
		timeObstructed = 0;
   }
}

void PIDController::reset() {
	lastSetPoint = 0;
	ITerm = 0;
	lastInput = 0;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
long PIDController::GetKp(){ return  dispKp; }
long PIDController::GetKi(){ return  dispKi;}
long PIDController::GetKd(){ return  dispKd;}
int PIDController::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PIDController::GetDirection(){ return controllerDirection;}
bool PIDController::GetObstructed(){ return isObstructed;}
bool PIDController::GetObstructionDetection(){ return enableObstructionDetetection;}	