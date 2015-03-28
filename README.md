# Arduino-PID-NoFloatingPoint
This PID library only supports the long data-type, preventing floating point operations, in order to speed up setpoint
calculations. This is a fork from the original Arduino PID library. Additionally, some small features like error margins and
obstruction detection have been added.

## Usage
```cpp
#include "PID_Extended_Controller.h"

/**
 * Initialization
 *
 * @param long P proportional gain
 * @param long I proportional gain
 * @param long D proportional gain
 * @param int Dir direction
 */
PIDController pid = PIDController(1, 0.1, 0.02, 0);

/**
 * Usage
 */
// Set control variables
long* InputAngle;
long* Output;
long* Setpoint;
InputAngle = new long(0.0);
Output = new long(0.0);
Setpoint = new long(0.0);
pid.SetControlVariables(long* InputAngle, long* Output, long* Setpoint)

// Update input variables
InputAngle = sensorReading();

// Compute PID output
pid.Compute();

// Use output
setMotorDutyCycle(*Output);
```

## Author
This library has been developed by [Wilbert van de Ridder](http://www.github.com/WRidder) for a BSc assignment at the [University of Twente](http://www.utwente.nl).